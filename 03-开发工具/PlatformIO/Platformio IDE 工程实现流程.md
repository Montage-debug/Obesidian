
| 快捷键        | 内容           |
| ---------- | ------------ |
| Ctrl+Alt+B | 编译工程         |
| Ctrl+Alt+U | 将程序上传烧录到开发板  |
| Ctrl+Alt+S | 打开串口 Monitor |

| 指令              | 作用                    | 备注      |
| --------------- | --------------------- | ------- |
| lsusb           | 查询当前已连接驱动             |         |
| ls /dev/ttyUSB* | 查询dev目录下所有ttyUSB开头的设备 | 星号*：通配符 |
[Ubuntu22.04 CH340系列串口芯片无法识别终极问题解决方案 | 鱼香ROS](https://fishros.org.cn/forum/topic/1050/ubuntu22-04-ch340%E7%B3%BB%E5%88%97%E4%B8%B2%E5%8F%A3%E8%8A%AF%E7%89%87%E6%97%A0%E6%B3%95%E8%AF%86%E5%88%AB%E7%BB%88%E6%9E%81%E9%97%AE%E9%A2%98%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%88)
[串口USB永久权限设置 | 鱼香ROS](https://fishros.org.cn/forum/topic/1150/%E4%B8%B2%E5%8F%A3usb%E6%B0%B8%E4%B9%85%E6%9D%83%E9%99%90%E8%AE%BE%E7%BD%AE)


| 函数原型 | 参数 | 返回值 | 描述 |
|---------|------|--------|------|
| `void begin(unsigned long baud)` | `baud`: 串口波特率 | `void` | 该函数用于初始化串口，主要配置串口波特率，波特率类似于频道号，串口收发双方保持相同的波特率才能进行正常通信。常见的波特率有9600，115200等，波特率其实代表每秒数据传输的频率，波特率越高，速度越快。 |
| `size_t printf(const char *format, ...)` | `format`: 格式化字符串 | `size_t`打印的字符数量 | 该函数和我们常见的`printf`函数一致，eg: `Serial.printf("Hello World!");` |
| `int read(void)` | `void` | `int`读取的字符的值，ASCII表示 | 该函数用于读取一个字节的数据，返回值就是这个字节的值，如果没有数据则返回 `-1` |

针对 Adafruit ESP32 Feather 开发板和 Arduino 框架，我来为您整理一份准确的安装和配置指南：

### 安装 PlatformIO 并创建 Adafruit ESP32 Feather 项目

#### **一、安装 PlatformIO Core**

1. **确保 Python 3.6+ 已安装**
   ```bash
   python3 --version  # Linux/macOS
   python --version   # Windows
   ```

2. **安装 PlatformIO Core**
   ```bash
   # Linux/macOS
   sudo pip3 install -U platformio
   
   # Windows
   pip install -U platformio
   ```

3. **验证安装**
   ```bash
   pio --version
   ```

#### **二、创建 Adafruit ESP32 Feather 项目**

1. **搜索正确的 Board ID**
   ```bash
   pio boards "adafruit feather esp32"
   ```
   从搜索结果中找到正确的 ID，通常是 `adafruit_feather_esp32`

2. **创建项目**
   ```bash
   # 进入项目目录
   cd ~/projects  # 替换为您的实际路径
   
   # 创建 Adafruit ESP32 Feather 项目，明确指定 Arduino 框架
   pio project init --board adafruit_feather_esp32 --framework arduino
   ```

#### **三、配置和代码**

1. **platformio.ini 配置文件**
   创建后，您的 `platformio.ini` 文件应该包含：
   ```ini
   [env:adafruit_feather_esp32]
   platform = espressif32
   board = adafruit_feather_esp32
   framework = arduino
   monitor_speed = 115200
   ```

2. **正确的 LED 闪烁代码**
   编辑 `src/main.cpp`，使用以下针对 ESP32 的正确代码：
   ```cpp
   #include <Arduino.h>

   // Adafruit ESP32 Feather 的内置 LED 引脚通常是 GPIO 13
   // 但不同版本可能不同，请根据实际硬件确认
   #define LED_BUILTIN 13

   void setup() {
     Serial.begin(115200);
     pinMode(LED_BUILTIN, OUTPUT);
     Serial.println("Adafruit ESP32 Feather LED Blink Started");
   }

   void loop() {
     digitalWrite(LED_BUILTIN, HIGH);
     Serial.println("LED ON");
     delay(1000);
     
     digitalWrite(LED_BUILTIN, LOW);
     Serial.println("LED OFF");
     delay(1000);
   }
   ```

   **重要提示**：如果 GPIO 13 不工作，请尝试以下引脚：
   - GPIO 2
   - GPIO 5
   - 或者查阅您的具体硬件文档

#### **四、编译和上传**

1. **编译项目**
   ```bash
   pio run
   ```

2. **上传到开发板**
   ```bash
   pio run --target upload
   ```

3. **查看串口输出**（可选）
   ```bash
   pio device monitor
   ```

#### **五、故障排除**

- **如果找不到板卡**：尝试更新 PlatformIO
  ```bash
  pio upgrade
  ```

- **如果上传失败**：
  1. 确保开发板已正确连接
  2. 检查 USB 数据线是否支持数据传输
  3. 可能需要手动按开发板上的 BOOT 按钮进入下载模式

- **如果 LED 不闪烁**：
  1. 确认正确的 LED 引脚号
  2. 检查 `platformio.ini` 配置是否正确
  3. 尝试不同的 GPIO 引脚

这个配置专门针对 Adafruit ESP32 Feather 开发板和 Arduino 框架，代码也针对 ESP32 进行了优化，包含了串口调试输出以便于 troubleshooting。