/*
 * Dallas' DS1820 family temperature sensor.
 * This library depends on the OneWire library (Dallas' 1-Wire bus protocol implementation)
 * available at <http://developer.mbed.org/users/hudakz/code/OneWire/>
 *
 * Example of use:
 * 
 * #include "DS1820.h"
 *
 * Serial serial(USBTX, USBRX);
 * 
 * int main() {
 *     DS1820  ds1820(PA_9);    // substitute PA_9 with actual mbed pin name connected to the DS1820 data pin
 *
 *     if(ds1820.begin()) {
 *        ds1820.startConversion();
 *        wait(1.0);
 *        while(1) {
 *            serial.printf("temp = %3.1f\r\n", ds1820.read());     // read temperature
 *            ds1820.startConversion();     // start temperature conversion
 *            wait(1.0);                    // let DS1820 complete the temperature conversion
 *        }
 *    } else
 *        serial.printf("No DS1820 sensor found!\r\n");
 * }
 *
 * 
 * Note: Don't forget to connect a 4.7k Ohm resistor 
 *       between the DS1820's data pin and the +3.3V pin
 *
 */
 
#include "DS1820.h"

#define DEBUG 1

#if DEBUG
//extern Serial serial;
extern int node_printf_to_serial(const char * format, ...); 
#endif

/**
 * @brief   Constructs a generic DS1820 sensor
 * @note    begin() must be called to detect and initialize the actual model
 * @param   pin: Name of data pin
 * @retval
 */
DS1820::DS1820(PinName pin) :
    oneWire(pin) {
    present = false;
    model_s = false;
}

/**
 * @brief   Constructs a specific model
 * @note    No need to call begin() to detect and initialize the model
 * @param   model:  One character model name: 'S', 's', 'B' or 'b'
 *          pin:    Name of data pin
 * @retval
 */
DS1820::DS1820(char model, PinName pin) :
    oneWire(pin) {
    if((model == 'S') or (model == 's')) {
        present = true;
        model_s = true;
    }
    else if((model == 'B') or (model == 'b')) {
        present = true;
        model_s = false;
    }
    else
        present = false;
}

/**
 * @brief   Detects and initializes the actual DS1820 model
 * @note
 * @param
 * @retval  true:   if a DS1820 family sensor was detected and initialized
            false:  otherwise
 */
bool DS1820::begin(void) {
    oneWire.reset_search();
    wait_ms(250);
    if(!oneWire.search(addr)) {
#if DEBUG
        node_printf_to_serial("No addresses.\r\n");
#endif
        oneWire.reset_search();
        wait_ms(250);
        return false;
    }

#if DEBUG
    node_printf_to_serial("ROM =");
    for(uint8_t i = 0; i < 8; i++) {
        node_printf_to_serial(" %x", addr[i]);
    }
    node_printf_to_serial("\r\n");
#endif

    if(OneWire::crc8(addr, 7) == addr[7]) {
        present = true;

        // the first ROM byte indicates which chip
        switch(addr[0]) {
        case 0x10:
            model_s = true;
#if DEBUG
            node_printf_to_serial("DS18S20 or old DS1820\r\n");
#endif            
            break;

        case 0x28:
            model_s = false;
#if DEBUG
            node_printf_to_serial("DS18B20\r\n");
#endif            
            break;

        case 0x22:
            model_s = false;
#if DEBUG
            node_printf_to_serial("DS1822\r\n");
#endif            
            break;

        default:
            present = false;
#if DEBUG
            node_printf_to_serial("Device doesn't belong to the DS1820 family\r\n");
#endif            
            return false;
        }
        return true;
    }
    else {
#if DEBUG    
        node_printf_to_serial("Invalid CRC!\r\n");
#endif    
        return false;
    }
}

/**
 * @brief   Informs about presence of a DS1820 sensor.
 * @note    begin() shall be called before using this function
 *          if a generic DS1820 instance was created by the user. 
 *          No need to call begin() for a specific DS1820 instance.
 * @param
 * @retval  true:   when a DS1820 sensor is present
 *          false:  otherwise
 */
bool DS1820::isPresent(void) {
    return present;
}

/**
 * @brief   Sets temperature-to-digital conversion resolution.
 * @note    The configuration register allows the user to set the resolution
 *          of the temperature-to-digital conversion to 9, 10, 11, or 12 bits.
 *          Defaults to 12-bit resolution for DS18B20.
 *          DS18S20 allows only 9-bit resolution.
 * @param   res:    Resolution of the temperature-to-digital conversion in bits.
 * @retval
 */
void DS1820::setResolution(uint8_t res) {
    // keep resolution within limits
    if(res > 12)
        res = 12;
    if(res < 9)
        res = 9;      
    if(model_s)
        res = 9;
       
    oneWire.reset();
    oneWire.skip();
    oneWire.write(0xBE);            // to read Scratchpad
    for(uint8_t i = 0; i < 9; i++)  // read Scratchpad bytes
        data[i] = oneWire.read();   

    data[4] |= (res - 9) << 5;      // update configuration byte (set resolution)  
    oneWire.reset();
    oneWire.skip();
    oneWire.write(0x4E);            // to write into Scratchpad
    for(uint8_t i = 2; i < 5; i++)  // write three bytes (2nd, 3rd, 4th) into Scratchpad
        oneWire.write(data[i]);
}

/**
 * @brief   Starts temperature conversion
 * @note    The time to complete the converion depends on the selected resolution:
 *           9-bit resolution -> max conversion time = 93.75ms
 *          10-bit resolution -> max conversion time = 187.5ms
 *          11-bit resolution -> max conversion time = 375ms
 *          12-bit resolution -> max conversion time = 750ms
 * @param
 * @retval
 */
void DS1820::startConversion(void) {
    if(present) {
        //node_printf_to_serial("reset\n\r");
        oneWire.reset();
        //node_printf_to_serial("skip\n\r");
        oneWire.skip();
        //node_printf_to_serial("write\n\r");
        oneWire.write(0x44);    //start temperature conversion
        //node_printf_to_serial("done\n\r");
    }
}

/**
 * @brief   Reads temperature from the chip's Scratchpad
 * @note
 * @param
 * @retval  Floating point temperature value
 */
float DS1820::read(void) {
    if(present) {
        //node_printf_to_serial("read: reset\n\r");
        oneWire.reset();
        //node_printf_to_serial("read: skip\n\r");
        oneWire.skip();
        //node_printf_to_serial("read: write\n\r");
        oneWire.write(0xBE);            // to read Scratchpad
        //node_printf_to_serial("read: read\n\r");
        for(uint8_t i = 0; i < 9; i++)  // read Scratchpad bytes
            data[i] = oneWire.read();   
        //node_printf_to_serial("read: read complete\n\r");

        // Convert the raw bytes to a 16-bit unsigned value
        uint16_t*   p_word = reinterpret_cast < uint16_t * > (&data[0]);

#if DEBUG
        //node_printf_to_serial("raw = %#x\r\n", *p_word);
#endif            

        if(model_s) {
            *p_word = *p_word << 3;         // 9-bit resolution
            if(data[7] == 0x10) {

                // "count remain" gives full 12-bit resolution
                *p_word = (*p_word & 0xFFF0) + 12 - data[6];
            }
        }
        else {
            uint8_t cfg = (data[4] & 0x60); // default 12-bit resolution
            
            // at lower resolution, the low bits are undefined, so let's clear them
            if(cfg == 0x00)
                *p_word = *p_word &~7;      //  9-bit resolution
            else
            if(cfg == 0x20)
                *p_word = *p_word &~3;      // 10-bit resolution
            else
            if(cfg == 0x40)
                *p_word = *p_word &~1;      // 11-bit resolution
                                               
        }

        // Convert the raw bytes to a 16-bit signed fixed point value :
        // 1 sign bit, 7 integer bits, 8 fractional bits (two’s compliment
        // and the LSB of the 16-bit binary number represents 1/256th of a unit).
        *p_word = *p_word << 4;
        
        // Convert to floating point value
        //node_printf_to_serial("read: return toFloat\n\r");
        return(toFloat(*p_word));
    }
    else {
        //node_printf_to_serial("read: return 0\n\r");
        return 0;
    }
}

/**
 * @brief   Converts a 16-bit signed fixed point value to floating point value
 * @note    The 16-bit unsigned integer represnts actually
 *          a 16-bit signed fixed point value:
 *          1 sign bit, 7 integer bits, 8 fractional bits (two’s compliment
 *          and the LSB of the 16-bit binary number represents 1/256th of a unit).       
 * @param   16-bit unsigned integer
 * @retval  Floating point value
 */
float DS1820::toFloat(uint16_t word) {
    if(word & 0x8000)
        return (-float(uint16_t(~word + 1)) / 256.0f);
    else
        return (float(word) / 256.0f);
}


