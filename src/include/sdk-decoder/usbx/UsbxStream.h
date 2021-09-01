/**
 * Copyright (c), 2017-2018 QuantumCTek. All rights reserved.
 * @file        : UsbxStream.h
 * @project     : QCTekMidware
 * @author      : John 
 * @date        : 2017-03-04
 * @version     : 1.0.0
 * @brief       :
 */
#ifndef __QCTEK_MIDWARE_USBX_STREAM_H
#define __QCTEK_MIDWARE_USBX_STREAM_H

#include <IODevice.h>
#include "Usbx.h"

namespace ss {
	/**
	 * libusb库Stream封装
	 */
	class __star_export UsbxStream : public Usbx, public IODevice {
	public:
		/**
		 * 构造函数
		 * @param vendor  USB设备的VID
		 * @param product USB设备的PID
		 * @param epIn    输入endpoint
		 * @param epOut   输出endpoint
		 */
		UsbxStream(uint16_t vendor, uint16_t product, uint8_t epIn, uint8_t epOut);

		/**
		 * 打开USB设备，见Usbx::open()
		 */
		void open();

		/**
		 * 关闭设备，见Usbx::close()
		 */
		void close();

		/**
		 * 从USB设备的输入endpoint读取数据
		 * @param buffer
		 * @param length
		 * @return 读取数据的长度
		 */
		ssize_t read(char *buffer, size_t length);

		/**
		 * 向USB设备的输出endpoint写入数据
		 * @param buffer
		 * @param length
		 * @return 实际写入的数据的长度
		 */
		ssize_t write(const char *buffer, size_t length);

		/**
		 * 获取USB设备的打开状态
		 * @return USB设备已经打开则返回true，否则返回false
		 */
		bool opened() const;

		/**
		 * 返回USB设备的名称
		 * @return libusb_VID_PID
		 */
		std::string who() const;

	protected:
		uint16_t _vendor;
		uint16_t _product;
		uint8_t  _epIn;
		uint8_t  _epOut;
	};

}

#endif //__QCTEK_MIDWARE_USBX_STREAM_H
