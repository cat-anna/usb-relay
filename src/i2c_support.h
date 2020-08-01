#pragma once

#include <TinyWireM.h> 

void i2c_start() {
    TinyWireM.begin();  
}

void i2c_scan(uint8_t *scan_result, uint8_t buffer_size) {
  TinyWireM.begin();   
  memset(scan_result, 0, buffer_size);
  for(uint8_t address = 1; address < 127; ++address )
  {
    TinyWireM.beginTransmission(address);
    uint8_t error = TinyWireM.endTransmission();

    if (error == 0)
    {
        uint8_t byte = (address >> 3) & 0xf;
        uint8_t bit = address & 0b111;
        if(buffer_size > byte) {
            scan_result[byte] |= (1 << bit);
        }
    }  
  }
}

enum BitOperation : uint8_t {
    BIT_OPERATION_NOP=0,
    BIT_OPERATION_WRITE_BIT=1,
    BIT_OPERATION_WRITE_VALUE=2,
    
    // BIT_OPERATION_SET=,
    // BIT_OPERATION_CLEAR=,
    // BIT_OPERATION_TOGGLE=,
};


class IOInterface {
public:
    virtual uint8_t read() = 0;
    virtual void output_write(uint8_t data) = 0;
    virtual uint8_t output_status() = 0;
};

class PCF8574 : public IOInterface {
public:
    PCF8574(uint8_t address): _address(address){}
    uint8_t read() override {
        if (TinyWireM.requestFrom(_address, (uint8_t)1) != 1)
        {
            return 0; 
        }
        return TinyWireM.read();
    }

    void output_write(uint8_t data) override {
        _status = data;
        TinyWireM.beginTransmission(_address);
        TinyWireM.write(~data);
        TinyWireM.endTransmission();
    }    
    uint8_t output_status() override  { return _status; }
private:
    uint8_t _address = 0;
    uint8_t _status = 0;
};

struct BitCommandData {
    IOInterface* device;
    BitOperation operation;
    uint8_t bit;
    uint8_t value;
};


void i2c_handle_operation(const BitCommandData *command) {
    uint8_t value = command->device->output_status();
    uint8_t bit_mask = 1 << (command->bit & 0b111);

    switch(command->operation) {
        case BIT_OPERATION_WRITE_BIT:
            if(command->value) {
                value |= bit_mask;
            } else {
                value &= ~bit_mask;
            }
        break;
        case BIT_OPERATION_WRITE_VALUE:
            value = command->value;
            break;
    default:
        return;
    }

    command->device->output_write(value);
}
