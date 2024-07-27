#include "mock_i2c_interface.h"
#include <stddef.h>

#define MAX_MOCK_REGISTER 4

// Define mock variables
static int mock_open_count = 0;
static int mock_set_slave_address_count = 0;
static int mock_write_byte_count = 0;
static int mock_read_byte_count = 0;
static int mock_close_count = 0;

static const char *mock_i2c_open_device = NULL;
static uint8_t mock_i2c_set_slave_address_addr = 0;
static uint8_t mock_i2c_write_byte_reg[MAX_MOCK_REGISTER] = { 0 };
static uint8_t mock_i2c_write_byte_val[MAX_MOCK_REGISTER] = { 0 };
static uint8_t mock_i2c_read_byte_reg[MAX_MOCK_REGISTER] = { 0 };
static uint8_t mock_i2c_read_byte_val[MAX_MOCK_REGISTER] = { 0xFF, 0xFF, 0xFF, 0xFF };
static int write_index = 0;
static int read_index = 0;

// Mock functions
static int mock_i2c_open(const char *device)
{
    mock_i2c_open_device = device;
    mock_open_count++;
    return 0; // Simulate success
}

static int mock_i2c_set_slave_address(uint8_t addr)
{
    mock_i2c_set_slave_address_addr = addr;
    mock_set_slave_address_count++;
    return 0; // Simulate success
}

static void mock_i2c_close()
{
    mock_close_count++;
}

static int mock_i2c_write_byte(uint8_t reg, uint8_t val)
{
    if (write_index < MAX_MOCK_REGISTER) {
        mock_i2c_write_byte_reg[write_index] = reg;
        mock_i2c_write_byte_val[write_index] = val;
        write_index++;
    } else {
        // Handle overflow or error
    }
    mock_write_byte_count++;
    return 0; // Simulate success
}

static uint8_t mock_i2c_read_byte(uint8_t reg)
{
    for (int i = 0; i < MAX_MOCK_REGISTER; i++) {
        if (mock_i2c_read_byte_reg[i] == reg) {
            mock_read_byte_count++;
            return mock_i2c_read_byte_val[i];
        }
    }
    mock_read_byte_count++;
    return 0xFF; // Default value if register is not found// Return a preset value
}

// Define the mock I2C interface
static I2CInterface mock_i2c_interface = { .open = mock_i2c_open,
                                           .set_slave_address = mock_i2c_set_slave_address,
                                           .close = mock_i2c_close,
                                           .write_byte = mock_i2c_write_byte,
                                           .read_byte = mock_i2c_read_byte };

// Set the mock I2C interface for testing
void set_mock_i2c_interface(void)
{
    set_i2c_interface(&mock_i2c_interface);
}

// Functions to access mock call counts or set return values
void reset_mock_i2c_counters(void)
{
    mock_open_count = 0;
    mock_set_slave_address_count = 0;
    mock_close_count = 0;
    mock_write_byte_count = 0;
    mock_read_byte_count = 0;
    write_index = 0;
    read_index = 0;
}

int get_mock_i2c_open_count(void)
{
    return mock_open_count;
}

int get_mock_i2c_set_slave_address_count(void)
{
    return mock_set_slave_address_count;
}

int get_mock_i2c_close_count(void)
{
    return mock_close_count;
}

int get_mock_i2c_write_byte_count(void)
{
    return mock_write_byte_count;
}

int get_mock_i2c_read_byte_count(void)
{
    return mock_read_byte_count;
}

const char *get_mock_i2c_open_device(void)
{
    return mock_i2c_open_device;
}

uint8_t get_mock_i2c_set_slave_addr(void)
{
    return mock_i2c_set_slave_address_addr;
}
uint8_t get_mock_i2c_write_byte_reg(int index)
{
    if (index < 4) {
        return mock_i2c_write_byte_reg[index];
    }
    return 0;
}
uint8_t get_mock_i2c_write_byte_val(int index)
{
    if (index < 4) {
        return mock_i2c_write_byte_val[index];
    }
    return 0;
}

uint8_t get_mock_i2c_read_byte_reg(int index)
{
    if (index < MAX_MOCK_REGISTER) {
        return mock_i2c_read_byte_reg[index];
    }
    return 0;
}

uint8_t get_mock_i2c_read_byte_val(int index)
{
    if (index < MAX_MOCK_REGISTER) {
        return mock_i2c_read_byte_val[index];
    }
    return 0;
}

void set_mock_i2c_read_byte_val(uint8_t reg, uint8_t val)
{
    for (int i = 0; i < MAX_MOCK_REGISTER; i++) {
        if (mock_i2c_read_byte_reg[i] == reg || mock_i2c_read_byte_reg[i] == 0) {
            mock_i2c_read_byte_reg[i] = reg;
            mock_i2c_read_byte_val[i] = val;
            break;
        }
    }
}
