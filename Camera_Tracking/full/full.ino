#include <Servo.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Servo servoX;
Servo servoY;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Khởi tạo màn hình LCD với địa chỉ I2C 0x27 và kích thước 16x2

StaticJsonDocument<200> jsonBuffer;  // Điều chỉnh kích thước buffer tùy theo dữ liệu JSON của bạn

void setup() {
  servoX.attach(9);  // Điều chỉnh chân servo cho servo X
  servoY.attach(10);  // Điều chỉnh chân servo cho servo Y
  Serial.begin(9600);  // Khởi tạo kết nối Serial với tốc độ 9600 bps
  lcd.init(); // Khởi tạo màn hình LCD
  lcd.backlight(); // Bật đèn nền LCD
  lcd.setCursor(0, 0); // Đặt con trỏ ở dòng 1, cột 0
  lcd.print("Variable 1:"); // In nội dung lên màn hình LCD
  lcd.setCursor(0, 1); // Đặt con trỏ ở dòng 2, cột 0
  lcd.print("Variable 2:");
}

void loop() {
  if (Serial.available() > 0) {
    // Đọc dữ liệu từ Serial
    String received_data = Serial.readStringUntil('\n');
    
    // Phân tách chuỗi thành hai số nguyên
    int angleX = received_data.substring(0, received_data.indexOf(',')).toInt();
    received_data = received_data.substring(received_data.indexOf(',') + 1);
    int angleY = received_data.substring(0, received_data.indexOf(',')).toInt();

    // Điều khiển servo X và Y
    servoX.write(angleX);
    servoY.write(angleY);

    // Hiển thị giá trị lên màn hình LCD
    lcd.setCursor(12, 0); // Đặt con trỏ ở dòng 1, cột 12
    lcd.print(angleX); // In giá trị angleX lên màn hình LCD
    lcd.setCursor(12, 1); // Đặt con trỏ ở dòng 2, cột 12
    lcd.print(angleY); // In giá trị angleY lên màn hình LCD
  }
}
