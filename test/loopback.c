#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DEV_PATH        "/dev/td3_i2s"
#define SYS_MASTER      "/sys/class/td3_i2s/master"
#define SYS_TX          "/sys/class/td3_i2s/transmitir"
#define SYS_RX          "/sys/class/td3_i2s/recibir"

#define SYS_READ_SIZE   4096U
#define SMALL_SIZE      8U
#define MEDIUM_SIZE     30U
#define OVERFLOW_SIZE   32768U
#define DEV_SLACK_SMALL 512U
#define DEV_SLACK_LARGE 2048U

typedef struct {
    int passed;
    int failed;
} test_summary_t;

typedef struct {
    size_t rx_len;
    int err;
    uint8_t buf[];
} fork_rx_shared_t;

static void hexdump_ascii(const char *label, const uint8_t *buf, size_t len) {
    size_t effective_len = len;

    /* Si termina en una cola larga de 0x00, recortar visualmente dejando contexto */
    if (len > 128) {
        size_t last_nonzero = 0;
        bool any_nonzero = false;
        for (size_t i = 0; i < len; i++) {
            if (buf[i] != 0x00) {
                last_nonzero = i;
                any_nonzero = true;
            }
        }
        if (!any_nonzero) {
            effective_len = 64;
        } else if (last_nonzero + 32 < len) {
            effective_len = (last_nonzero + 32 + 15) & ~((size_t)15);
            if (effective_len > len)
                effective_len = len;
        }
    }

    printf("%s (%zu bytes):\n", label, len);
    for (size_t i = 0; i < effective_len; i += 16) {
        printf("  %04zx: ", i);
        for (size_t j = 0; j < 16; j++) {
            if (i + j < effective_len)
                printf("%02x ", buf[i + j]);
            else
                printf("   ");
            if (j == 7)
                putchar(' ');
        }
        printf(" |");
        for (size_t j = 0; j < 16 && (i + j) < effective_len; j++) {
            uint8_t c = buf[i + j];
            putchar((c >= 32 && c <= 126) ? (char)c : '.');
        }
        printf("|\n");
    }
    if (effective_len < len) {
        printf("  ... [%zu bytes finales en 0x00 omitidos]\n", len - effective_len);
    }
}

static void fill_unique_ascii(uint8_t *buf, size_t len, unsigned int seed, const char *tag) {
    char header[32];
    size_t pos = 0;
    unsigned int seq = 0;

    if (len <= 12) {
        static const char alphabet[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
        unsigned int state = seed ^ (unsigned int)tag[0];
        for (size_t i = 0; i < len; i++) {
            state = state * 1103515245U + 12345U;
            buf[i] = (uint8_t)alphabet[(state >> 16) % (sizeof(alphabet) - 1)];
        }
        return;
    }

    while (pos < len) {
        int n = snprintf(header, sizeof(header), "[%04X-%s-#%04X]", seed & 0xFFFFU, tag, seq++);
        if (n <= 0)
            break;
        size_t hdr_len = (size_t)n;
        size_t chunk = (len - pos < hdr_len) ? (len - pos) : hdr_len;
        memcpy(buf + pos, header, chunk);
        pos += chunk;
    }
}

static ssize_t find_pattern_offset(const uint8_t *haystack, size_t hlen,
                                   const uint8_t *needle, size_t nlen) {
    if (nlen == 0 || hlen < nlen)
        return -1;
    for (size_t i = 0; i <= hlen - nlen; i++) {
        if (memcmp(haystack + i, needle, nlen) == 0)
            return (ssize_t)i;
    }
    return -1;
}

/*
 * Busca el inicio de TX dentro de RX permitiendo desplazamiento (ej. ceros de idle).
 * Intenta primero con un prefijo de 8 bytes (o tx_len si es menor), y si los primeros
 * bytes tuvieron algún glitch, busca cualquier bloque posterior de TX y retrocalcula
 * dónde debería empezar TX[0] en RX.
 */
static bool locate_shifted_start(const uint8_t *tx, size_t tx_len,
                                 const uint8_t *rx, size_t rx_len,
                                 ssize_t *rx_start_out, size_t *tx_start_out) {
    size_t probe = (tx_len < 8) ? tx_len : 8;
    ssize_t off = find_pattern_offset(rx, rx_len, tx, probe);
    if (off >= 0) {
        *rx_start_out = off;
        *tx_start_out = 0;
        return true;
    }

    /* Fallback: buscar fragmentos posteriores de TX de 8 bytes dentro de RX */
    if (tx_len >= 16 && rx_len >= 8) {
        for (size_t t_off = 4; t_off + 8 <= tx_len && t_off <= 256; t_off += 4) {
            off = find_pattern_offset(rx, rx_len, tx + t_off, 8);
            if (off >= 0) {
                if ((size_t)off >= t_off) {
                    *rx_start_out = off - (ssize_t)t_off;
                    *tx_start_out = 0;
                } else {
                    *rx_start_out = 0;
                    *tx_start_out = t_off - (size_t)off;
                }
                return true;
            }
        }
    }

    return false;
}

static int set_master_mode(const char *val) {
    int fd = open(SYS_MASTER, O_WRONLY);
    if (fd < 0) {
        perror("open " SYS_MASTER);
        return -1;
    }
    if (write(fd, val, strlen(val)) < 0) {
        perror("write " SYS_MASTER);
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

static ssize_t write_all(int fd, const uint8_t *buf, size_t len, int *err_out) {
    size_t total = 0;
    while (total < len) {
        ssize_t w = write(fd, buf + total, len - total);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            if (err_out)
                *err_out = errno;
            return (total > 0) ? (ssize_t)total : -1;
        }
        if (w == 0)
            break;
        total += (size_t)w;
    }
    if (err_out)
        *err_out = 0;
    return (ssize_t)total;
}

static ssize_t read_all(int fd, uint8_t *buf, size_t len, int *err_out) {
    size_t total = 0;
    while (total < len) {
        ssize_t r = read(fd, buf + total, len - total);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            if (err_out)
                *err_out = errno;
            return (total > 0) ? (ssize_t)total : -1;
        }
        if (r == 0)
            break;
        total += (size_t)r;
    }
    if (err_out)
        *err_out = 0;
    return (ssize_t)total;
}

static fork_rx_shared_t *alloc_shared_rx(size_t rx_cap, size_t *map_size_out) {
    size_t map_size = sizeof(fork_rx_shared_t) + rx_cap;
    fork_rx_shared_t *shm = mmap(NULL, map_size, PROT_READ | PROT_WRITE,
                                 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (shm == MAP_FAILED)
        return NULL;
    memset(shm, 0, map_size);
    *map_size_out = map_size;
    return shm;
}

static void evaluate_shifted_and_report(const char *name,
                                        const uint8_t *tx, size_t tx_len,
                                        const uint8_t *rx, size_t rx_len,
                                        bool dump_full,
                                        test_summary_t *summary) {
    ssize_t rx_start = 0;
    size_t tx_start = 0;
    bool found;

    printf("\n------------------------------------------------------------\n");
    printf("[CASE] %s | TX=%zu bytes | RX=%zu bytes\n", name, tx_len, rx_len);

    if (dump_full) {
        hexdump_ascii("TX", tx, tx_len);
        hexdump_ascii("RX", rx, rx_len);
    }

    found = locate_shifted_start(tx, tx_len, rx, rx_len, &rx_start, &tx_start);
    if (!found) {
        printf("  [DIAG] No se encontro el inicio ni fragmentos de la secuencia TX dentro de RX\n");
        if (!dump_full && rx_len > 0) {
            hexdump_ascii("RX (primeros 128 B)", rx, rx_len < 128 ? rx_len : 128);
        }
        printf("[FAIL] %s: secuencia TX ausente en RX\n", name);
        summary->failed++;
        return;
    }

    printf("  [DIAG] Inicio alineado: TX[%zu] <-> RX[%zd] (desplazamiento RX = %+zd bytes)\n",
           tx_start, rx_start, rx_start - (ssize_t)tx_start);

    size_t rem_tx = tx_len - tx_start;
    size_t rem_rx = rx_len - (size_t)rx_start;
    size_t cmp_len = (rem_tx < rem_rx) ? rem_tx : rem_rx;
    size_t mismatches = 0;
    size_t shown = 0;

    for (size_t i = 0; i < cmp_len; i++) {
        uint8_t tb = tx[tx_start + i];
        uint8_t rb = rx[(size_t)rx_start + i];
        if (tb != rb) {
            mismatches++;
            if (shown < 12) {
                uint8_t tc = (tb >= 32 && tb <= 126) ? tb : '.';
                uint8_t rc = (rb >= 32 && rb <= 126) ? rb : '.';
                printf("  [DIFF @TX+%zu / RX+%zu] TX=0x%02x('%c') vs RX=0x%02x('%c')\n",
                       tx_start + i, (size_t)rx_start + i, tb, (char)tc, rb, (char)rc);
                shown++;
            }
        }
    }

    size_t missing_tail = (tx_start + cmp_len < tx_len) ? (tx_len - (tx_start + cmp_len)) : 0;
    if (missing_tail > 0) {
        printf("  [DIAG] Faltaron %zu bytes al final de RX por el desplazamiento inicial\n",
               missing_tail);
    }
    if (tx_start > 0) {
        printf("  [DIAG] Se perdieron los primeros %zu bytes de TX antes de RX[0]\n", tx_start);
    }

    if (mismatches == 0 && tx_start == 0 && cmp_len == tx_len) {
        printf("[PASS] %s: %zu/%zu bytes coinciden tras alinear en RX+%zd (0 fallos)\n",
               name, cmp_len, tx_len, rx_start);
        summary->passed++;
    } else {
        size_t total_bad = mismatches + tx_start + missing_tail;
        printf("[FAIL] %s: comparados=%zu, corruptos=%zu, perdidos_inicio=%zu, faltantes_fin=%zu (%.2f%% fallo total)\n",
               name, cmp_len, mismatches, tx_start, missing_tail,
               tx_len > 0 ? (100.0 * (double)total_bad / (double)tx_len) : 100.0);
        summary->failed++;
    }
}

static void run_dev_small_or_med(const char *name, const char *tag, size_t size,
                                 unsigned int seed, test_summary_t *summary) {
    size_t rx_cap = size + DEV_SLACK_SMALL;
    size_t map_size = 0;
    uint8_t *tx = malloc(size);
    fork_rx_shared_t *shm = alloc_shared_rx(rx_cap, &map_size);
    int ready_pipe[2];
    pid_t pid;
    int err = 0;

    if (!tx || !shm) {
        fprintf(stderr, "[ERROR] Sin memoria en %s\n", name);
        free(tx);
        if (shm)
            munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pipe(ready_pipe) < 0) {
        perror("pipe");
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    fill_unique_ascii(tx, size, seed, tag);

    int fd = open(DEV_PATH, O_RDWR);
    if (fd < 0) {
        perror("open " DEV_PATH);
        close(ready_pipe[0]);
        close(ready_pipe[1]);
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    pid = fork();
    if (pid < 0) {
        perror("fork");
        close(fd);
        close(ready_pipe[0]);
        close(ready_pipe[1]);
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pid == 0) {
        /* Proceso hijo: lector (read) */
        int rx_err = 0;
        char ready = 'R';

        close(ready_pipe[0]);
        (void)write(ready_pipe[1], &ready, 1);
        close(ready_pipe[1]);

        ssize_t r = read_all(fd, shm->buf, size, &rx_err);
        size_t total_r = (r > 0) ? (size_t)r : 0;

        /*
         * Si por los 0x00 de idle previos al write la secuencia quedó desplazada,
         * leer margen adicional para capturar la secuencia completa en RX.
         */
        ssize_t rx_start = 0;
        size_t tx_start = 0;
        while (total_r < rx_cap) {
            if (locate_shifted_start(tx, size, shm->buf, total_r, &rx_start, &tx_start) &&
                tx_start == 0 && (size_t)rx_start + size <= total_r) {
                break;
            }
            size_t chunk = (rx_cap - total_r < 64) ? (rx_cap - total_r) : 64;
            ssize_t extra = read_all(fd, shm->buf + total_r, chunk, &rx_err);
            if (extra <= 0)
                break;
            total_r += (size_t)extra;
        }

        close(fd);
        shm->rx_len = total_r;
        shm->err = rx_err;
        _exit(0);
    }

    /* Proceso padre: escritor (write) */
    close(ready_pipe[1]);
    char ready_byte = 0;
    (void)read(ready_pipe[0], &ready_byte, 1);
    close(ready_pipe[0]);

    usleep(10000);

    ssize_t w = write_all(fd, tx, size, &err);
    if (w < 0 || (size_t)w != size) {
        fprintf(stderr, "[ERROR] %s write=%zd/%zu (errno=%d: %s)\n",
                name, w, size, err, strerror(err));
    }

    int wstatus = 0;
    waitpid(pid, &wstatus, 0);
    close(fd);

    evaluate_shifted_and_report(name, tx, (w > 0 ? (size_t)w : 0),
                                shm->buf, shm->rx_len, true, summary);

    free(tx);
    munmap(shm, map_size);
}

static void run_dev_overflow_32k(unsigned int seed, test_summary_t *summary) {
    const char *name = "/dev/td3_i2s (32 KB - desborde de buffer 16 KB)";
    size_t rx_cap = OVERFLOW_SIZE + DEV_SLACK_LARGE;
    size_t map_size = 0;
    uint8_t *tx = malloc(OVERFLOW_SIZE);
    fork_rx_shared_t *shm = alloc_shared_rx(rx_cap, &map_size);
    int ready_pipe[2];
    pid_t pid;
    int err = 0;

    if (!tx || !shm) {
        fprintf(stderr, "[ERROR] Sin memoria en %s\n", name);
        free(tx);
        if (shm)
            munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pipe(ready_pipe) < 0) {
        perror("pipe");
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    fill_unique_ascii(tx, OVERFLOW_SIZE, seed, "D32K");

    int fd = open(DEV_PATH, O_RDWR);
    if (fd < 0) {
        perror("open " DEV_PATH);
        close(ready_pipe[0]);
        close(ready_pipe[1]);
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    pid = fork();
    if (pid < 0) {
        perror("fork");
        close(fd);
        close(ready_pipe[0]);
        close(ready_pipe[1]);
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pid == 0) {
        /* Proceso hijo: lector (read) */
        int rx_err = 0;
        char ready = 'R';

        close(ready_pipe[0]);
        (void)write(ready_pipe[1], &ready, 1);
        close(ready_pipe[1]);

        ssize_t r = read_all(fd, shm->buf, rx_cap, &rx_err);
        close(fd);

        shm->rx_len = (r > 0) ? (size_t)r : 0;
        shm->err = rx_err;
        _exit(0);
    }

    /* Proceso padre: escritor (write) */
    close(ready_pipe[1]);
    char ready_byte = 0;
    (void)read(ready_pipe[0], &ready_byte, 1);
    close(ready_pipe[0]);

    usleep(10000);

    ssize_t w = write_all(fd, tx, OVERFLOW_SIZE, &err);
    if (w < 0 || (size_t)w != OVERFLOW_SIZE) {
        fprintf(stderr, "[ERROR] %s write=%zd/%u (errno=%d: %s)\n",
                name, w, OVERFLOW_SIZE, err, strerror(err));
    }

    int wstatus = 0;
    waitpid(pid, &wstatus, 0);
    close(fd);

    size_t tx_len = (w > 0) ? (size_t)w : 0;
    evaluate_shifted_and_report(name, tx, tx_len, shm->buf, shm->rx_len, false, summary);

    free(tx);
    munmap(shm, map_size);
}

static void run_sys_case(const char *name, const char *tag, size_t tx_size,
                         unsigned int seed, test_summary_t *summary) {
    size_t map_size = 0;
    uint8_t *tx = malloc(tx_size);
    fork_rx_shared_t *shm = alloc_shared_rx(SYS_READ_SIZE, &map_size);
    int ready_pipe[2];
    pid_t pid;
    int err = 0;

    if (!tx || !shm) {
        fprintf(stderr, "[ERROR] Sin memoria en %s\n", name);
        free(tx);
        if (shm)
            munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pipe(ready_pipe) < 0) {
        perror("pipe");
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    fill_unique_ascii(tx, tx_size, seed, tag);

    /* Limpiar rx_buf previo abriendo y cerrando /dev antes del caso /sys */
    int fd_clean = open(DEV_PATH, O_RDWR);
    if (fd_clean >= 0)
        close(fd_clean);

    pid = fork();
    if (pid < 0) {
        perror("fork");
        close(ready_pipe[0]);
        close(ready_pipe[1]);
        free(tx);
        munmap(shm, map_size);
        summary->failed++;
        return;
    }

    if (pid == 0) {
        /* Proceso hijo: lector (sysfs recibir) */
        int fd_rx;
        int rx_err = 0;
        char ready = 'R';

        close(ready_pipe[0]);
        fd_rx = open(SYS_RX, O_RDONLY);
        if (fd_rx < 0) {
            perror("child open " SYS_RX);
            shm->err = errno;
            (void)write(ready_pipe[1], &ready, 1);
            close(ready_pipe[1]);
            _exit(1);
        }

        (void)write(ready_pipe[1], &ready, 1);
        close(ready_pipe[1]);

        ssize_t r = read_all(fd_rx, shm->buf, SYS_READ_SIZE, &rx_err);
        if (r < 0) {
            fprintf(stderr, "[ERROR] %s read=%zd (errno=%d: %s)\n",
                    name, r, rx_err, strerror(rx_err));
            r = 0;
        }
        close(fd_rx);

        shm->rx_len = (size_t)r;
        shm->err = rx_err;
        _exit(0);
    }

    /* Proceso padre: escritor (sysfs transmitir) */
    close(ready_pipe[1]);
    char ready_byte = 0;
    (void)read(ready_pipe[0], &ready_byte, 1);
    close(ready_pipe[0]);

    usleep(10000);

    int fd_tx = open(SYS_TX, O_WRONLY);
    ssize_t w = -1;
    if (fd_tx < 0) {
        perror("parent open " SYS_TX);
    } else {
        w = write_all(fd_tx, tx, tx_size, &err);
        if (w < 0 || (size_t)w != tx_size) {
            fprintf(stderr, "[ERROR] %s write=%zd/%zu (errno=%d: %s)\n",
                    name, w, tx_size, err, strerror(err));
        }
        close(fd_tx);
    }

    int wstatus = 0;
    waitpid(pid, &wstatus, 0);

    uint8_t *rx = shm->buf;
    size_t rx_len = shm->rx_len;
    size_t tx_len = (w > 0) ? (size_t)w : 0;

    if (tx_size <= SYS_READ_SIZE) {
        evaluate_shifted_and_report(name, tx, tx_len, rx, rx_len, true, summary);
    } else {
        /* En /sys con 32 KB enviados, rx_buf (16 KB circular) conserva el tramo final */
        printf("\n------------------------------------------------------------\n");
        printf("[CASE] %s | TX=%zu bytes | RX=%zu bytes (ventana sysfs 4 KB)\n",
               name, tx_len, rx_len);
        hexdump_ascii("RX (primeros 256 B)", rx, rx_len < 256 ? rx_len : 256);

        size_t rx_nz = 0;
        while (rx_nz < rx_len && rx[rx_nz] == 0x00)
            rx_nz++;

        size_t probe_len = 16;
        ssize_t match_in_tx = (rx_nz + probe_len <= rx_len)
                              ? find_pattern_offset(tx, tx_len, rx + rx_nz, probe_len)
                              : -1;

        if (match_in_tx >= 0) {
            size_t avail_tx = tx_len - (size_t)match_in_tx;
            size_t avail_rx = rx_len - rx_nz;
            size_t cmp_len = (avail_tx < avail_rx) ? avail_tx : avail_rx;
            size_t mismatches = 0;
            size_t shown = 0;

            for (size_t i = 0; i < cmp_len; i++) {
                uint8_t tb = tx[(size_t)match_in_tx + i];
                uint8_t rb = rx[rx_nz + i];
                if (tb != rb) {
                    mismatches++;
                    if (shown < 12) {
                        uint8_t tc = (tb >= 32 && tb <= 126) ? tb : '.';
                        uint8_t rc = (rb >= 32 && rb <= 126) ? rb : '.';
                        printf("  [DIFF @TX+%zu / RX+%zu] TX=0x%02x('%c') vs RX=0x%02x('%c')\n",
                               (size_t)match_in_tx + i, rx_nz + i, tb, (char)tc, rb, (char)rc);
                        shown++;
                    }
                }
            }

            printf("  [DIAG] RX[%zu..] alinea con TX[%zd..] (comparados=%zu bytes)\n",
                   rx_nz, match_in_tx, cmp_len);

            if (mismatches == 0 && cmp_len > 0) {
                printf("[PASS] %s: %zu/%zu bytes del tramo coinciden sin errores\n",
                       name, cmp_len, cmp_len);
                summary->passed++;
            } else {
                printf("[FAIL] %s: %zu/%zu bytes fallaron (%.2f%% error)\n",
                       name, mismatches, cmp_len,
                       cmp_len > 0 ? (100.0 * (double)mismatches / (double)cmp_len) : 100.0);
                summary->failed++;
            }
        } else {
            printf("  [DIAG] No se encontro ningun bloque de 16 B de RX dentro de TX\n");
            printf("[FAIL] %s: ventana de 4 KB corrupta o ausente\n", name);
            summary->failed++;
        }
    }

    free(tx);
    munmap(shm, map_size);
}

int main(void) {
    unsigned int base_seed = (unsigned int)time(NULL) ^ ((unsigned int)getpid() << 8);
    test_summary_t summary = {0, 0};

    printf("=== Diagnostico Loopback td3_i2s (Seed=0x%08X) ===\n", base_seed);

    if (set_master_mode("1") < 0)
        return 1;
    printf("[INIT] Modo MASTER seteado correctamente\n");

    /* --- Parte 1: /dev/td3_i2s --- */
    printf("\n==================== FASE 1: /dev/td3_i2s ====================\n");
    run_dev_small_or_med("/dev/td3_i2s (8 bytes - pocos datos)", "D8B",
                         SMALL_SIZE, base_seed + 1U, &summary);
    run_dev_small_or_med("/dev/td3_i2s (30 bytes - datos medianos)", "D30",
                         MEDIUM_SIZE, base_seed + 2U, &summary);
    run_dev_overflow_32k(base_seed + 3U, &summary);

    /* --- Parte 2: /sys/class/td3_i2s/ --- */
    printf("\n================= FASE 2: /sys/class/td3_i2s =================\n");
    run_sys_case("/sys transmitir/recibir (8 bytes -> 4 KB)", "S8B",
                 SMALL_SIZE, base_seed + 4U, &summary);
    run_sys_case("/sys transmitir/recibir (30 bytes -> 4 KB)", "S30",
                 MEDIUM_SIZE, base_seed + 5U, &summary);
    run_sys_case("/sys transmitir/recibir (32 KB -> 4 KB)", "S32",
                 OVERFLOW_SIZE, base_seed + 6U, &summary);

    /* Restaurar modo SLAVE por seguridad electrica al finalizar */
    set_master_mode("0");

    printf("\n============================================================\n");
    printf("RESUMEN: %d casos PASS | %d casos FAIL\n", summary.passed, summary.failed);
    printf("============================================================\n");

    return (summary.failed == 0) ? 0 : 1;
}