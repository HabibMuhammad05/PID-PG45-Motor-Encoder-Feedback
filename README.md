Last Updated: 11/05/25

github Percobaan motor PG45 dengan internal encoder +PID controller
https://github.com/HabibMuhammad05/PID-PG45-Motor-Encoder-Feedback

Penjelasan tiap sketch program:

- Motor_With_Encoder  : Pembacaan A&B Internal Encoder, Output pulse untuk kontrol Motor.
	https://github.com/HabibMuhammad05/PID-PG45-Motor-Encoder-Feedback/tree/master/Motor_With_Encoder
	
- MotorControl_RPM_Tuning.ino  : PID 1 motor searah, Hanya CW.
	https://github.com/HabibMuhammad05/PID-PG45-Motor-Encoder-Feedback/tree/master/MotorControl_RPM_Tuning
	
- MotorControl_PID_DualDirection.ino  : Perhitungan & tuning PID 1 motor support pergerakan 2 arah (CW & CCW).
	https://github.com/HabibMuhammad05/PID-PG45-Motor-Encoder-Feedback/tree/master/MotorControl_PID_DualDirection
	
- MotorControl_PID_MultiMotor.ino  : Kontrol 4 motor support pergerakan 2 arah (CW&CCW), Perhitungan sendiri-sendiri per motor.
	https://github.com/HabibMuhammad05/PID-PG45-Motor-Encoder-Feedback/tree/master/MotorControl_PID_MultiMotor
