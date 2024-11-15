# BK4819 Arduino Library

<img src="images/board X.jpg" alt="Board X" style="display: block; margin-left: auto; margin-right: auto; width: 50%;">
<img src="images/if radio.png" alt="Board X" style="display: block; margin-left: auto; margin-right: auto; width: 50%;">

## Description
This is an Arduino library for interfacing with the **Beken BK4819** radio module. The library allows easy control and communication with the **BK4819 tool**, providing functions to handle various radio operations.

## Features
- Full control of BK4819 radio functionalities.
- Easy integration with Arduino projects.
- Example sketches provided for quick start.

## Installation
To install the library:

1. Download the repository as a ZIP file.
2. Open the Arduino IDE.
3. Go to **Sketch** > **Include Library** > **Add .ZIP Library...**.
4. Select the downloaded ZIP file to add the library.

## Getting Started
### Basic Usage
Include the library in your sketch:

```cpp
#include <BK4819.h>

BK4819 beken(10, 11, 12, 13);   // Passa i pin CS, MOSI, MISO, e SCK

void setup() 
{
    beken.BK4819_Init(); 
}
```

## Examples
The library comes with several example sketches to help you get started. You can find them in the **examples** folder of the library:

- **Radio_base**: Demonstrates how to set the Bk4819 module.

To load an example, go to **File** > **Examples** > **BK4819** in the Arduino IDE.

## Dependencies
This library requires the following additional libraries:
- **IcomSYM**: To handle CI-V protocol commands.
- **TaskScheduler**: For managing asynchronous tasks in the examples.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author
- **Fabrizio Palumbo (IU0IJV)**
- Contact: [t.me/IU0IJV](https://t.me/IU0IJV)

## Contributions
Contributions are welcome! Please feel free to submit pull requests to improve the library or fix issues.