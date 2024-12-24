# Reggi

Reggi is a project designed for remote control using the CRSF (Crossfire) protocol. It provides libraries and implementations for both transmitter (TX) and receiver (RX) components.

## Features

- **CRSF Protocol Support**: Implements key functionalities for communication via the CRSF protocol.
- **Modular Design**: Includes separate libraries for flexible integration.
- **Optimized for Embedded Systems**: Written in C++ and C for performance and portability.

## Repository Structure

- `Lib/`: Contains shared libraries used by both TX and RX components.
- `Reggi TX/`: Code for the transmitter module.
- `Reggi RX/`: Code for the receiver module.

## Getting Started

### Prerequisites

- A development environment with support for C and C++.
- Hardware supporting the CRSF protocol (e.g., compatible radios and receivers).

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/VohminV/Reggi.git
   cd Reggi
   ```

2. Navigate to the desired module directory (`Reggi TX` or `Reggi RX`) and follow the build instructions in the respective README files.

### Usage

1. Integrate the libraries from the `Lib` directory into your project.
2. Configure the TX and RX modules to match your hardware setup.
3. Deploy the compiled code to your hardware.

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request with a detailed description of your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Special thanks to the open-source community and the creators of the CRSF protocol for making this project possible.

