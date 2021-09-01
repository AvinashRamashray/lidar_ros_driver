/**
* Copyright (c), 2017-2018 QuantumCTek. All rights reserved.
* @file        : Usbx.h
* @project     : QCTekMidware
* @author      : John
* @date        : 2017-03-04
* @version     : 1.0.0
* @brief       :
*/
#ifndef __USBX_H
#define __USBX_H

#include <star/Star.h>
#include "IODevice.h"

#if __LIBUSB_LEGACY
struct usb_dev_handle;
#else
struct libusb_device_handle;
struct libusb_context;
#endif

namespace ss {

	class __star_export UsbxException {
	public:
		explicit UsbxException(const std::string &what);
		UsbxException(const std::string &device, const std::string &operate);
		UsbxException(const std::string &device, const std::string &operate, const std::string &message);
	};

	/**
	 * libusb库封装，API和libusb-1.0 API和libusb-0.1的兼容API，通过宏__LIBUSB_LEGACY开关控制
	 */
	class __star_export Usbx {
	public:
		Usbx();
		virtual ~Usbx();

		/**
		 * 打开USB设备
		 * @param vendor 设备的VID
		 * @param product 设备的PID
		 */
		void open(uint16_t vendor, uint16_t product);

		/**
		 * 关闭USB设备
		 * 如果设备未成功打开则什么都不做
		 */
		void close() throw();

		/**
		 * 从USB设备读取数据
		 * @param endpoint 读取数据的endpoint
		 * @param buffer
		 * @param length
		 * @return 实际读取的长度，失败则抛出UsbxException
		 */
		size_t receive(uint8_t endpoint, char *buffer, size_t length);

		/**
		 * 向USB设备写入数据
		 * @param endpoint 写入数据的endpoint
		 * @param buffer
		 * @param length
		 * @return 实际写入数据的长度，失败则抛出UsbxException
		 */
		size_t send(uint8_t endpoint, const char *buffer, size_t length);

		/**
		 * 返回当前读取缓冲区中的数据大小，未实现，返回0
		 * @param endpoint
		 * @return 未实现，总是返回0
		 */
		size_t available(uint8_t endpoint);

		/**
		 * USB设备的打开状态
		 * @return 设备成功打开则返回true，否则返回false
		 */
		bool opened() const;

	private:
#if 0
		uint16_t _vendor;
		uint16_t _product;
		uint8_t _epIn;
		uint8_t _epOut;
#endif

#if __LIBUSB_LEGACY
		usb_dev_handle       *_dev;
#else
		libusb_device_handle *_dev;
		libusb_context       *_ctx;
#endif
		//static unsigned int _init_count;
	};

}

#endif //__USBX_H
