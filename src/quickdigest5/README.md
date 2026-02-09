# QuickDigest5

QuickDigest5 is a lightweight C++ library for computing MD5 hashes of strings and files. Designed for simplicity and efficiency, it provides static utility methods to generate MD5 hashes and convert them to hexadecimal string representations.

- Compute MD5 hashes for strings and files.
- Generate hashes in binary (vector of bytes) or hexadecimal string format.
- Lightweight and dependency-free, leveraging the C++ Standard Library.
- Designed as a final class to ensure security and encapsulation.

## Usage

Here's a simple example program that computes the MD5 hash of one or more files:

```cpp
/**
 * This program accepts one or more file paths as command-line arguments. 
 * For each file, it computes its MD5 hash using the `QuickDigest5::fileToHash` 
 * method and prints the result to the console. If no file paths are provided, 
 * it displays a usage message indicating how to run the program.
 */
#include <iostream>
#include <quickdigest5.hpp>

int main(int argc, char** argv) {
    if(argc > 1)
        // Process each file path provided as an argument.
        for(int i = 1; i < argc; ++i)
            std::cout << QuickDigest5::fileToHash(argv[i])
                << std::endl;
    // Display usage instructions if no arguments are provided.
    else std::cout << "Usage:" << std::endl
        << "\t" << argv[0] << " <file-to-hash>"
        << std::endl;

    return 0;
}
```

## API Overview

1. Hashing Strings

```cpp
std::string hash = QuickDigest5::toHash("example string");
```

2. Hashing Files

```cpp
std::string fileHash = QuickDigest5::fileToHash("example.txt");
```

3. Binary Hash (Vector of Bytes)

```cpp
std::vector<uint8_t> binaryHash = QuickDigest5::digestString("example string");
std::vector<uint8_t> fileBinaryHash = QuickDigest5::digestFile("example.txt");
```

## Contributing

Contributions are welcome! Feel free to submit a pull request or open an issue for bugs or feature requests.

## License

QuickDigest5 is licensed under the [GNU General Public License v3](License). You are free to use, modify, and distribute this library under the terms of the license.
