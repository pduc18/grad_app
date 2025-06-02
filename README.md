# Cấu hình các chân trong CubeMX:
	+ Clock 72MHz
	+ PA8, PA9 Timer 1 chế độ encoder dùng để đọc encoder (đọc cả 2 kênh, lên và xuống, autoreload) -> quay 1 vòng trục chính 11(pulses)x56x4(sườn lên) = 1232 pulse/round
	+ PA6: PWM, PSC = 3, ARR = 999 => f =~ 18kHz -> ảnh hưởng của tần số?
	+ PA4, PA5: output điều khiển hướng dòng điện cho driver
	+ Ngắt Timer2 mỗi 1ms
	+ UART1 + DMA để xử lý dữ liệu từ Pi truyền về (tách gói, đem x, y để tra Dtf)\
	+ I2C để log dữ liệu lên LCD
	+ UART2 để log dữ liệu lên máy tính
	
# Motor JGB37-545 12VDC 107rpm
	+ Ratio: 56:1

	st-flash --reset write build grad_app.bin 0x8000000
