// Enable SPI Interface
`define SPI_INTERFACE

// Enable Buffer DTR pin
//`define BUFFER_DTR

// Enable Move Done Pin
//`define MOVE_DONE

// Motor Definitions
//`define DUAL_HBRIDGE 1
`define ULTIBRIDGE 1

// Encoder Count
`define QUAD_ENC 1

// Change the Move Buffer Size. Should be power of two
//`define MOVE_BUFFER_SIZE 4
// Version Number specification following SemVer
`define VERSION_MAJOR 0
`define VERSION_MINOR 1
`define VERSION_PATCH 0

// Default move buffer if not specified
`ifndef MOVE_BUFFER_SIZE
`define MOVE_BUFFER_SIZE 2
`endif

`define MOVE_BUFFER_BITS $clog2(`MOVE_BUFFER_SIZE) - 1 // number of bits to index given size

`define MOTOR_COUNT DUAL_HBRIDGE // + other supported topologies in the future.
