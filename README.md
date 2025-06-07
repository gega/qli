# qli
QLI is Quite Light Image format inspired by QOI

QLI is a lightweight, header-only lossless image codec based on [QOI](https://qoiformat.org/), extended for embedded and low memory use cases. It is designed to be portable, customizable, and friendly for both file-based and memory-based image workflows. The decoder is aimed to be used in embedded environment while the encoder is more relaxed, designed for desktop use.

## Hightlights

* Streaming support
* Static configuration
* Alpha channel removed
* Multiple pixel formats (`RGBA`, `RGB565`, etc.)
* Big- and little-endian support
* Optional stride handling
* Customizable index size
* No dynamic allocation or stdio if not needed
* Streamed decoding for low memory usage

## Basic API Usage

### Encoding to file

```c
#include "rppm.h"

struct rppm img;
rppm_load(&img, "sample.ppm");
qli_save(img.pixels, img.width, img.height, "sample.qli");
```

### Decoding from memory buffer

```c

struct qli_image qli;
uint8_t buffer[100];

qli_init( &qli, width, height, width * QLI_BPP, data, data_size );

while( 0 < ( decoded_bytes = qli_decode(&qli, buffer, sizeof(buffer))))
{
  // Handle decoded_bytes of buffer
}
```

### Decoding from file

```c
struct qli_image qli;
uint8_t header[QLI_HEADER_LEN];

fread(header, 1, QLI_HEADER_LEN, fp);
qli_init_header(&qli, header, file_size);

// Use the same decoding loop as above
```

---

## Configuration Options

Define these macros **before** including `qli.h` to tailor the library to your needs:

| Macro                | Description                                                            |
| -------------------- | ---------------------------------------------------------------------- |
| `QLI_NOSTDIO`        | Exclude all file I/O functionality (`qli_save`, `qli_init_header`)     |
| `QLI_PIXEL_FORMAT`   | Sets the default pixel format (e.g., `QLI_RGB565`)                     |
| `QLI_USERDATA`       | Enables `void *userdata` field in the `qli_state` struct               |
| `QLI_POSTFIX`        | Appends a custom postfix to all symbol names (for namespacing)         |
| `QLI_ENDIAN`         | Overrides platform endianness: `0 = little`, `1 = big`                 |
| `QLI_INDEX_SIZE`     | Sets the size of the index array (values: `16`, `32`, `64`, etc.)      |
