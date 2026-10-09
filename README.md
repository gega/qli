# qli
QLI is Quite Light Image format inspired by QOI

QLI is a lightweight, header-only lossless image codec based on [QOI](https://qoiformat.org/), extended for embedded and low memory use cases. It is designed to be portable, customizable, and friendly for both file-based and memory-based image workflows. The decoder is aimed to be used in embedded environment while the encoder is more relaxed, designed for desktop use.
Pixel formats are for both encoding and decoding - the same configuration needed for both encoding and decoding. If multiple configurations needed, use the `QLI_POSTFIX` to define multiple decoders/encoders without name collision. Pixel formats are defining the format of the output buffers, in case of `RGB444` the output buffer contains two pixels in every three bytes so the output buffer should be divisible by three and the emitted number of pixels are always even. Similarly for other formats, the buffer needs to be divisible by the pixel width to accomodate whole pixels.

## Hightlights

* Fast streaming support
* Static configuration
* Multiple pixel formats (`RGB888`, `RGB565`, `RGB444`)
* Big- and little-endian output support
* Customizable index size
* No dynamic allocation or stdio if not needed
* Streamed decoding for low memory usage

## Basic API Usage

### Encoding to file
_using  [rppm](https://github.com/gega/rppm) library_
```c
#include "rppm.h"

struct rppm img;
rppm_load(&img, "sample.ppm");
qli_save(img.pixels, img.width, img.height, "sample.qli");
```

### Decoding from memory buffer

```c

qli_image_t qli;
uint8_t buffer[QLI_MIN_OUTPUT_BUFFER_SIZE * 34];

int flags;
int32_t bytes_written;
int consumed;

if (qli_init(&qli, width, height, data, data_size, 0) != 0)
    return -1;

for (;;) {
    consumed = qli_decode(&qli, buffer, sizeof(buffer),
                          &flags, &bytes_written);

    // Handle bytes_written bytes of decoded pixels
    if (bytes_written > 0) {
        process_pixels(buffer, bytes_written);
    }

    if (flags & QLI_RF_END_OF_STREAM)
        break;

    if (flags & QLI_RF_MORE_DATA) {
        // Fetch the next compressed input chunk
        if (!fetch_next_chunk(&new_data_ptr, &new_data_size))
            return -1; // Truncated input

        qli_new_chunk(&qli, new_data_ptr, new_data_size);
    }
}
```

### Decoding from file

```c
qli_image_t qli;
uint8_t header[QLI_HEADER_LEN];

fread(header, 1, QLI_HEADER_LEN, fp);
qli_init_header(&qli, header, 0);

// Use the same decoding loop as above
```

---

## Configuration Options

Define these macros **before** including `qli.h` to tailor the library to your needs:

| Macro                | Description                                                                       |
| -------------------- | --------------------------------------------------------------------------------- |
| `QLI_NOSTDIO`        | Exclude all file I/O functionality if 1 (`qli_save`, `qli_init_header`)           |
| `QLI_PIXEL_FORMAT`   | Sets the default pixel format (`QLI_PF_RGB565`, `QLI_PF_RGB444`, `QLI_PF_RGB888`) |
| `QLI_USERDATA`       | Add extra fields in the `qli_image` struct                                        |
| `QLI_POSTFIX`        | Appends a custom postfix to all symbol names (for namespacing)                    |
| `QLI_ENDIAN`         | Set output endianness (`QLI_LITTLE_ENDIAN`, `QLI_LITTLE_ENDIAN`)                  |
| `QLI_INDEX_SIZE`     | Sets the bit width of the index array (values: `6`, `5`, `4`, etc.)               |
| `QLI_STRIDE`         | Define as 1 to enable stride support                                              |
| `QLI_DEBUG`          | Define as 1 or 2 to enable printing out the encoded/decoded opcodes and more      |
