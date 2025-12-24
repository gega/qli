# qli
QLI is Quite Light Image format inspired by QOI

QLI is a lightweight, header-only lossless image codec based on [QOI](https://qoiformat.org/), extended for embedded and low memory use cases. It is designed to be portable, customizable, and friendly for both file-based and memory-based image workflows. The decoder is aimed to be used in embedded environment while the encoder is more relaxed, designed for desktop use.

## Hightlights

* Streaming support
* Static configuration
* Alpha channel removed
* Multiple pixel formats (`RGB888`, `RGB565`, `RGB444`)
* Big- and little-endian support
* Optional stride handling
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

struct qli_image qli;
uint8_t buffer[100];

qli_init( &qli, width, height, width * QLI_BPP, data, data_size ); // data_size can be smaller than the compressed image but must be even

while( 0 < ( decoded_pixels = qli_decode(&qli, buffer, sizeof(buffer), &new_chunk)))
{
  // Handle decoded_bytes of buffer
  if(new_chunk)
  {
    new_chunk = 0;
    // fetch new set of data and update buffers:
    qli_new_chunk(&qli, new_data_ptr, new_data_size);
  }
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

| Macro                | Description                                                                       |
| -------------------- | --------------------------------------------------------------------------------- |
| `QLI_NOSTDIO`        | Exclude all file I/O functionality (`qli_save`, `qli_init_header`)                |
| `QLI_PIXEL_FORMAT`   | Sets the default pixel format (`QLI_PF_RGB565`, `QLI_PF_RGB444`, `QLI_PF_RGB888`) |
| `QLI_USERDATA`       | Add extra fields in the `qli_image` struct                                        |
| `QLI_POSTFIX`        | Appends a custom postfix to all symbol names (for namespacing)                    |
| `QLI_ENDIAN`         | Overrides platform endianness (`QLI_LITTLE_ENDIAN`, `QLI_LITTLE_ENDIAN`)          |
| `QLI_INDEX_SIZE`     | Sets the size of the index array (values: `16`, `32`, `64`, etc.)                 |
| `QLI_STRIDE`         | Define as 1 to enable stride support                                              |
| `QLI_DEBUG`          | Define as 1 to enable printing out the encoded/decoded opcodes                    |
