# intelligent-closestool
一款实用stm32f0芯片,采用st公司的HAL库及freeRTos为基础来开发的智能马桶主板软件,(包含软件硬件).采用市场较为流行的即热式模块.

# pcb
pcb部分使用easyeda https://lceda.cn/weiqqba/ji-re-shi-mai-chong-fa-chong-shui-zhi-neng-ma-tong
这是此次的电路板,部分电阻值可能没用修改,发现时修改.各位也可以给我留言

#app flutter
手机app进去后修改conBan.dart页面的


var _BTid = /*"98:5D:AD:1D:57:5A"*/ "00:15:83:00:AB:00" ;

如果你不知道的模块的id,可以联机运行一下,软件会print扫描到的蓝牙的id,然后改进去就可以运行了.
然后健康数据传输过去的时间暂时会不准,暂时不知道是不是手机设置问题.也没打算很详细去改正,准备后面以正常的遥控+wifi联网,然后控制和健康数据分开走,并加入个人习惯的温度,风温等更加详细的记录.

#其他内容见wiki