/*
 * rosserial_config.h
 *
 *  Created on: Aug 29, 2025
 *      Author: 88698
 */

#ifndef ROSSERIAL_CONFIG_H_
#define ROSSERIAL_CONFIG_H_

// Don't redefine MAX_SUBSCRIBERS/MAX_PUBLISHERS as they're template parameters
// Instead, configure buffer sizes only

// Conservative buffer sizes for stability
#define SERIAL_BUFFER_SIZE 512
#define INPUT_SIZE 512  
#define OUTPUT_SIZE 512

// Longer timeouts for better reliability
#define TIMEOUT_MS 2000

// Enable debug mode (comment out in production)
//#define ROSSERIAL_DEBUG

#endif /* ROSSERIAL_CONFIG_H_ */
