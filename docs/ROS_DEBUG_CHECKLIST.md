# ROS Serial Communication Debug Checklist

## Current Issue: Bad Checksum Errors
The STM32 and ROS are experiencing communication errors with repeated "bad checksum" warnings.

## System Configuration ✅
- **STM32 UART**: USART3 at 115200 baud
- **ROS Serial**: 115200 baud 
- **Buffer Sizes**: STM32 = 4096 bytes, ROS = 4096 bytes (✅ **MATCHED**)
- **DMA**: Enabled for both TX and RX

## Debug Steps to Try:

### 1. Verify Physical Connection
```bash
# Check if STM32 is detected
ls /dev/ttyUSB* /dev/ttyACM*

# Test raw serial communication
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0 -b 115200
```

### 2. Test with Reduced Message Rate
Edit ros1.cpp to reduce publishing frequency:
```cpp
// In ROS update loop, add delay
static uint32_t last_publish = 0;
if(HAL_GetTick() - last_publish > 100) {  // 100ms = 10Hz instead of faster
    // Publish messages here
    last_publish = HAL_GetTick();
}
```

### 3. Check Message Sizes
Add debug output to see message lengths:
```cpp
// In STM32Hardware.h write function
void write(uint8_t* data, int length){
    // Add this debug line temporarily
    // printf("TX: %d bytes\n", length);
    // ... rest of function
}
```

### 4. Verify ROS Launch Command
Ensure you're using the correct port and baud:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

### 5. Test with Minimal Messages
Temporarily disable some publishers to isolate the issue:
```cpp
// Comment out some publishers in ros1.cpp
// pub_cmd_vel.publish(&cmd_vel_msg);
// pub_status.publish(&status_msg);
```

### 6. Check for Ground Loops
- Ensure STM32 and PC share common ground
- Try different USB ports/cables
- Test with USB isolator if available

### 7. Monitor with Logic Analyzer
If available, check the actual UART signals for:
- Correct baud rate
- Proper start/stop bits
- Signal integrity

## Expected Results:
After implementing rosserial_config.h, you should see:
- Reduced checksum errors
- More stable communication
- Successful message parsing

## Next Steps if Issues Persist:
1. Implement message rate limiting
2. Add debug prints to identify problematic messages
3. Test with single message type at a time
4. Consider hardware-level debugging
