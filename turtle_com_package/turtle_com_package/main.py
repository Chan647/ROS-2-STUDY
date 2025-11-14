from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from db_helper import DB, DB_CONFIG
import sys
import time
import roslibpy
import threading

class KeyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Turtle Controller")
        self.setWindowIcon(QIcon("/home/chan/ros2_study/src/turtle_com_package/turtle_com_package/turtle.png"))
        self.db = DB(**DB_CONFIG)

        hbox = QHBoxLayout()
        vbox = QVBoxLayout()
        grid = QGridLayout()

        btn_w = QPushButton("W")
        btn_a = QPushButton("A")
        btn_s = QPushButton("S")
        btn_d = QPushButton("D")
        btn_style = """ QPushButton {background-color: #ffffff; border-radius: 15px; border: 2px solid #000000; font-size: 18px; font-weight: bold;} 
        QPushButton:hover {background-color: #e6e6e6; border: 2px solid #b3b3b3;} QPushButton:pressed {background-color: #d6d6d6;}"""
        btn_w.setStyleSheet(btn_style)
        btn_a.setStyleSheet(btn_style)
        btn_s.setStyleSheet(btn_style)
        btn_d.setStyleSheet(btn_style)
                            
        btn_w.clicked.connect(lambda: self.send_com('w'))
        btn_a.clicked.connect(lambda: self.send_com('a'))
        btn_s.clicked.connect(lambda: self.send_com('s'))
        btn_d.clicked.connect(lambda: self.send_com('d'))


        for btn in [btn_w, btn_a, btn_s, btn_d]:
            btn.setFixedSize(60, 60)

        grid.addWidget(btn_w, 0, 1)   
        grid.addWidget(btn_a, 1, 0)   
        grid.addWidget(btn_s, 1, 1)   
        grid.addWidget(btn_d, 1, 2)  


        btn_re = QPushButton("Reset")
        btn_l = QPushButton("Load")
        btn_re.setFixedSize(190, 130)
        btn_l.setFixedSize(190, 130)
        btn_re.setStyleSheet(btn_style)
        btn_l.setStyleSheet(btn_style)
        btn_re.clicked.connect(lambda: self.send_com('r'))
        btn_l.clicked.connect(self.add_con)

        btn_color = QPushButton("Change Background Color")
        btn_color.setFixedHeight(50)
        btn_color.clicked.connect(self.change_color)
        color_style = (
                        "QPushButton { "
                        "background-color: qlineargradient("
                        "spread:pad, x1:0, y1:0.5, x2:1, y2:0.5, "
                        "stop:0 red, stop:0.16 orange, stop:0.33 yellow, "
                        "stop:0.5 green, stop:0.66 blue, stop:0.83 indigo, stop:1 violet"
                        ");"
                        "border: 2px solid #000000; "
                        "border-radius:12px; "
                        "font-size:20px; "
                        "font-weight:bold;"
                        "} "
                        "QPushButton:hover { "
                        "border:2px solid white; "
                        "} "
                        "QPushButton:pressed { "
                        "background-color: qlineargradient("
                        "spread:pad, x1:0, y1:0.5, x2:1, y2:0.5, "
                        "stop:0 #cc0000, stop:0.16 #cc6600, stop:0.33 #cccc00, "
                        "stop:0.5 #009900, stop:0.66 #0000cc, stop:0.83 #660099, stop:1 #9900cc"
                        ");"
                        "} "
                    )
        btn_color.setStyleSheet(color_style)

        self.table = QTableWidget()
        self.table.setStyleSheet("QTableWidget { border: 1.5px solid #000000; gridline-color: #000000; } QHeaderView::section { background-color: #f0f0f0; font-weight: bold; font-size: 10pt; }");
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["ID", "X", "Y", "THETA", "TIME"])
        self.table.setEditTriggers(self.table.NoEditTriggers) 
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        hbox.addLayout(grid)
        hbox.addWidget(btn_re, alignment=Qt.AlignBottom)
        hbox.addWidget(btn_l, alignment=Qt.AlignBottom)

        vbox.addLayout(hbox)
        vbox.addWidget(btn_color)
        vbox.addWidget(self.table)

        self.setLayout(vbox)

        t = threading.Thread(target=self.ros)
        t.daemon = True
        t.start()

    def change_color(self):
        title = QDialog(self)
        title.setWindowTitle("Background Color Change")

        form = QFormLayout()
        self.input_red = QLineEdit()
        self.input_green = QLineEdit()
        self.input_blue = QLineEdit()

        label_red = QLabel("Red Value")
        label_green = QLabel("Green Value")
        label_blue = QLabel("Blue Value")
        label_red.setStyleSheet("background-color: #ff3333;" "border: 2px solid #000000;" "border-radius: 8px;" "padding: 4px 10px;""font-weight: bold;")
        label_green.setStyleSheet("background-color: #00cc66;" "border: 2px solid #000000;" "border-radius: 8px;" "padding: 4px 10px;""font-weight: bold;")
        label_blue.setStyleSheet("background-color: #3399ff;" "border: 2px solid #000000;" "border-radius: 8px;" "padding: 4px 10px;""font-weight: bold;")

        form.addRow(label_red,self.input_red)
        form.addRow(label_green,self.input_green)
        form.addRow(label_blue,self.input_blue)

        btn_ok = QPushButton("전송")
        btn_ok.setFixedHeight(30)
        btn_ok.clicked.connect(self.send_color)
        btn_bck = QPushButton("뒤로가기")
        btn_bck.clicked.connect(title.reject)
        btn_bck.setFixedHeight(30)
        
        btn_box = QHBoxLayout()
        btn_box.addWidget(btn_ok)
        btn_box.addWidget(btn_bck)

        root = QVBoxLayout()
        root.addLayout(form)
        root.addLayout(btn_box)
        title.setLayout(root)
        
        title.exec()
    
    def add_con(self):
        if self.time_str == "":
            QMessageBox.warning(self, "경고", "데이터가 수신되지 않았습니다.")
            return
    
        self.db.insert_con(self.x, self.y, self.theta, self.time_str)
        self.add_table()

    def add_table(self):
        row = self.table.rowCount()
        self.table.insertRow(row)
        
        self.table.setItem(row, 0, QTableWidgetItem(str(row+1)))      
        self.table.setItem(row, 1, QTableWidgetItem(str(self.x)))    
        self.table.setItem(row, 2, QTableWidgetItem(str(self.y)))    
        self.table.setItem(row, 3, QTableWidgetItem(str(self.theta))) 
        self.table.setItem(row, 4, QTableWidgetItem(self.time_str))
        
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.Fixed)
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        self.table.setColumnWidth(0, 30)
        self.table.setColumnWidth(4, 130)

    def ros(self):
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        def callback(msg):
            self.x = msg['x']
            self.y = msg['y']
            self.theta = msg['theta']
            self.time_str = msg['timestr']
            
            print("x:", self.x)
            print("y:", self.y)
            print("theta:", self.theta)
            print("time:", self.time_str)

        listener = roslibpy.Topic(self.client, '/turtle_time', 'turtle_com_package_msgs/TurtleMsg')
        listener.subscribe(callback)
        self.vel = roslibpy.Topic(self.client, 'turtle1/cmd_vel', 'geometry_msgs/Twist')
        self.reset = roslibpy.Service(self.client, 'reset', 'std_srvs/Empty')
        
    def send_com(self, cmd):
        forward = roslibpy.Message({'linear': {'x': 2.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
        backward = roslibpy.Message({'linear': {'x': -2.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
        left = roslibpy.Message({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 1.57}})
        right = roslibpy.Message({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': -1.57}})

        if cmd == 'w':
            self.vel.publish(forward)
        elif cmd == 'a':
            self.vel.publish(left)
        elif cmd == 's':
            self.vel.publish(backward)
        elif cmd == 'd':
            self.vel.publish(right)
        elif cmd == 'r':
            self.reset.call(roslibpy.ServiceRequest({}))

        time.sleep(0.2)
    
    def send_color(self):
        self.red = roslibpy.Param(self.client, '/turtlesim:background_r')
        self.green = roslibpy.Param(self.client, '/turtlesim:background_g')
        self.blue = roslibpy.Param(self.client, '/turtlesim:background_b')

        r = self.input_red.text().strip()
        g = self.input_green.text().strip()
        b = self.input_blue.text().strip()
        
        if r != "":
            try:
                int_r = int(r)
            except:
                QMessageBox.critical(self, "Error", "Red 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")
                return
            if 0 <= int_r <=255:
                self.red.set(r)
            else:
                QMessageBox.warning(self, "Over the Limit", "Red 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")
            
        if g != "":
            try:
                int_g = int(g)
            except:
                QMessageBox.critical(self, "Error", "Green 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")
                return
            if 0 <= int_g <=255:
                self.green.set(g)
            else:
                QMessageBox.warning(self, "Over the Limit", "Green 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")
        
        if b != "":
            try:
                int_b = int(b)
            except:
                QMessageBox.critical(self, "Error", "Blue 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")
                return
            if 0 <= int_b <=255:
                self.blue.set(b)
            else:
                QMessageBox.warning(self, "Over the Limit", "Blue 값에 0 ~ 255 사이의 숫자를 입력하십시오!!!")


if __name__ == "__main__":
     app = QApplication(sys.argv)
     w = KeyWidget()
     w.show()
     sys.exit(app.exec_())



