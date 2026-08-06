#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

enum {
    INPUT_CAPACITY = 6144,
    OUTPUT_CAPACITY = (INPUT_CAPACITY / 3) * 4,
};

static const char ALPHABET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int write_all(const char *data, size_t size) {
    while (size > 0) {
        ssize_t written = write(STDOUT_FILENO, data, size);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        data += written;
        size -= (size_t)written;
    }
    return 0;
}

static size_t encode_complete(const uint8_t *input, size_t size, char *output) {
    size_t input_offset = 0;
    size_t output_offset = 0;
    while (input_offset + 3 <= size) {
        uint32_t value = ((uint32_t)input[input_offset] << 16) |
                         ((uint32_t)input[input_offset + 1] << 8) |
                         (uint32_t)input[input_offset + 2];
        output[output_offset] = ALPHABET[(value >> 18) & 0x3f];
        output[output_offset + 1] = ALPHABET[(value >> 12) & 0x3f];
        output[output_offset + 2] = ALPHABET[(value >> 6) & 0x3f];
        output[output_offset + 3] = ALPHABET[value & 0x3f];
        input_offset += 3;
        output_offset += 4;
    }
    return output_offset;
}

int main(void) {
    uint8_t input[INPUT_CAPACITY + 2];
    char output[OUTPUT_CAPACITY + 4];
    size_t pending = 0;

    for (;;) {
        ssize_t received = read(STDIN_FILENO, input + pending, INPUT_CAPACITY - pending);
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            return 1;
        }
        if (received == 0) {
            break;
        }

        size_t total = pending + (size_t)received;
        size_t complete = total - total % 3;
        size_t encoded = encode_complete(input, complete, output);
        if (encoded > 0 && write_all(output, encoded) != 0) {
            return 2;
        }
        pending = total - complete;
        if (pending > 0) {
            memmove(input, input + complete, pending);
        }
    }

    if (pending == 1) {
        uint32_t value = (uint32_t)input[0] << 16;
        output[0] = ALPHABET[(value >> 18) & 0x3f];
        output[1] = ALPHABET[(value >> 12) & 0x3f];
        output[2] = '=';
        output[3] = '=';
        return write_all(output, 4) == 0 ? 0 : 2;
    }
    if (pending == 2) {
        uint32_t value = ((uint32_t)input[0] << 16) | ((uint32_t)input[1] << 8);
        output[0] = ALPHABET[(value >> 18) & 0x3f];
        output[1] = ALPHABET[(value >> 12) & 0x3f];
        output[2] = ALPHABET[(value >> 6) & 0x3f];
        output[3] = '=';
        return write_all(output, 4) == 0 ? 0 : 2;
    }
    return 0;
}
