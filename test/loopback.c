#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

static void print_buf(const unsigned char *buf, ssize_t n) {
    printf("Leidos %zd bytes:\n", n);
    for (ssize_t i = 0; i < n; i++) {
        if (buf[i] >= 32 && buf[i] <= 126)
            putchar(buf[i]);
        else
            printf("\\x%02x", buf[i]);
    }
    putchar('\n');
}

int main(void) {
    const char *msg = "Hola Mundo!";
    unsigned char buf[1024];
    ssize_t n;

    // Configurar modo MASTER al inicio (antes de todo)
    int fd_master = open("/sys/class/td3_i2s/master", O_WRONLY);
    if (fd_master < 0) { perror("open master"); return 1; }
    if (write(fd_master, "1", 1) < 0) { perror("write master"); close(fd_master); return 1; }
    close(fd_master);
    printf("[INIT] Modo MASTER seteado correctamente\n");

    // --- Seccion 1: /dev/td3_i2s ---
    printf("\n=== Prueba con /dev/td3_i2s ===\n");
    int fd = open("/dev/td3_i2s", O_RDWR);
    if (fd < 0) { perror("open /dev/td3_i2s"); return 1; }

    if (write(fd, msg, strlen(msg)) < 0) { perror("write /dev/td3_i2s"); close(fd); return 1; }
    printf("Se lograron enviar los datos\n");
    usleep(500); // espera ~0.5 ms a que salga y vuelva por I2S

    n = read(fd, buf, sizeof(buf));
    if (n < 0) { perror("read /dev/td3_i2s"); close(fd); return 1; }
    print_buf(buf, n);
    close(fd);

    // --- Seccion 2: /sys/class/td3_i2s/ ---
    printf("\n=== Prueba con /sys/class/td3_i2s/ ===\n");
    unsigned char tx_big[4200];
    size_t msg_len = strlen(msg);
    for (size_t i = 0; i < sizeof(tx_big); i++) {
        tx_big[i] = (unsigned char)msg[i % msg_len];
    }

    int fd_tx = open("/sys/class/td3_i2s/transmitir", O_WRONLY);
    if (fd_tx < 0) { perror("open transmitir"); return 1; }

    size_t total_sent = 0;
    while (total_sent < sizeof(tx_big)) {
        ssize_t w = write(fd_tx, tx_big + total_sent, sizeof(tx_big) - total_sent);
        if (w < 0) { perror("write transmitir"); close(fd_tx); return 1; }
        if (w == 0) break;
        total_sent += (size_t)w;
    }
    printf("Se lograron enviar %zu bytes por /sys\n", total_sent);
    close(fd_tx);

    usleep(500);

    int fd_rx = open("/sys/class/td3_i2s/recibir", O_RDONLY);
    if (fd_rx < 0) { perror("open recibir"); return 1; }

    unsigned char rx_big[4096];
    n = read(fd_rx, rx_big, sizeof(rx_big));
    if (n < 0) { perror("read recibir"); close(fd_rx); return 1; }
    print_buf(rx_big, n);
    close(fd_rx);

    return 0;
}