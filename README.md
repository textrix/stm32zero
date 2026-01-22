# STM32ZERO

Zero-Overhead C++ Utility Library for STM32

## Design Principles

- **Zero-overhead**: Extensive use of templates, constexpr, and inline
- **No runtime cost**: No virtual functions, RTTI, heap, or dynamic polymorphism
- **Modern Embedded C++**: C-like C++ style
- **Static allocation**: All resources statically allocated

## Code Style

- **Indentation**: Tab size 8
- **Enum names**: UPPERCASE (e.g., `Priority::NORMAL`, `Priority::HIGH`)
- **Naming**: snake_case (functions, variables), PascalCase (classes, enums)

## Requirements

- C++17 or later
- STM32 HAL Drivers

## License

MIT License
