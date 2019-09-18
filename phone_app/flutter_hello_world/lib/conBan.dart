import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_blue/flutter_blue.dart';
import 'dart:async';
import 'package:sqflite/sqflite.dart';


class ControlBan extends StatefulWidget {
  @override
  _CtrlBanState createState() {
    // TODO: implement createState
    return _CtrlBanState();
  }
}

class _CtrlBanState extends State<ControlBan> {
  FlutterBlue _BTIndi;
  BluetoothDevice _BTDevice;
  BluetoothCharacteristic _BTCharacter;
  Timer _timerShowIco;
  Database jkDB;
  Future getBT() async {
    var conBanStorge = await SharedPreferences.getInstance();
    conBanStorge.get("bt_key");
  }
  List<int> _vauleBuf=List();
  void loadData() async {
    var databasePath = await getDatabasesPath();
    String path = databasePath + "jk";
    jkDB = await openDatabase(path,
        version: 1,
        onCreate: (Database db, int version) async {
          await db.execute('''
      create table jkList(
      id integer primary key not null,
      tick integer,
      do integer,
      do_tick integer,
      hg_tick integer,
      sit_tick integer,
      exit_tick integer
      )
      ''');
          await db.close();
        }
    );
  }
  void makeFlashIco(){
    if(wifiColor==Colors.black){
      setState(() {
        wifiColor=Colors.white;
      });
    }else {
      setState(() {
        wifiColor = Colors.black;
      });
    }
  }
  @override
  void initState() {
    // TODO: implement initState
    super.initState();
    loadData();
    //print("initstate");
    _timerShowIco= Timer.periodic(Duration(milliseconds: 500), (timer){
      makeFlashIco();
    });

    blueToothConect();
  }
  @override
  void dispose() {
    // TODO: implement dispose
    _BTIndi.stopScan();
    //scanSubscription.cancel();
    if (_BTDevice != null) {
      _BTDevice.disconnect();
      _BTDevice=null;
    }
      print("stop scan BT!");
    if(_timerShowIco.isActive)_timerShowIco.cancel();
    if(_getJKList.isActive)_getJKList.cancel();
      super.dispose();
    }

  var _BTid = /*"98:5D:AD:1D:57:5A"*/ "00:15:83:00:AB:00" ;
  double waterTemplate = 30; //实际温度
  var sitTemplate = 1; //挡位
  double pjPos = 9; //挡位,位置max
  double fsfPos = 9; //开到最大
  int fwPos = 5; //风温,挡位
  bool ledOn = true;
  bool message = true; //按摩
  bool hotWaterOn=false;
  Color ledColor = Colors.red;
  Color hotWater=Colors.white;
  Color wifiColor=Colors.black;
  int fwValue=0;
  var fwIco=Icon(Icons.cancel) ;
  int zwValue=0;
  Timer _getJKList;
  //bool showWifi=true;
  void _buleTootchGetChar() async {
    if (_BTDevice == null) return;
    await _BTDevice.connect();
    List<BluetoothService> services = await _BTDevice.discoverServices();
    services.forEach((service) {
      // do something with service
      print(
          " has find the BT's service ${service.uuid},${service.characteristics.length}");
      if (service.uuid.toString().indexOf('ffe0') > 0) {
        List<BluetoothCharacteristic> chareacts = service.characteristics;
        chareacts.forEach((chare) {
          //print(
              //' 0xffe0 \'s characteristic ${chare.uuid},descript is ${chare.serviceUuid} ');
          //print("find ffe1 ${chare.serviceUuid.toString().indexOf("ffe1")} at postion");
          if (chare.uuid.toString().indexOf("ffe1") > 0) {
            _BTCharacter = chare;

            print("has find it!");

            chare.setNotifyValue(true);
            //_BTCharacter.
            _BTCharacter.value.listen((value) {
              if(value.length ==12) {
                //收到健康数据
                if(_vauleBuf.length!=20) return;
                value.forEach((v){
                  _vauleBuf.add(v);
                });
                print("jk");
                print(_vauleBuf);
                if(value[4]==00) _getJKList.cancel();
                if(jkDB.isOpen){
                  jkDB.insert("jkList", {'do': _vauleBuf[4],
                    'tick': _vauleBuf[24] + (_vauleBuf[25] >> 8) + (_vauleBuf[26] >> 16) +
                        (_vauleBuf[27] >> 24),
                    'do_tick': _vauleBuf[12] + (_vauleBuf[13] >> 8) +
                        (_vauleBuf[14] >> 16) + (_vauleBuf[15] >> 24),
                    'hg_tick': _vauleBuf[16] + (_vauleBuf[17] >> 8) +
                        (_vauleBuf[18] >> 16) + (_vauleBuf[19] >> 24),
                    'sit_tick': _vauleBuf[8] + (_vauleBuf[9] >> 8) + (_vauleBuf[10] >> 16) +
                        (_vauleBuf[11] >> 24),
                    'exit_tick': _vauleBuf[20] + (_vauleBuf[21] >> 8) +
                        (_vauleBuf[22] >> 16) + (_vauleBuf[23] >> 24)
                  });
                }else {
                  print("some error");
                }
                _vauleBuf.clear();
                //_vauleBuf.removeRange(0, end)
              }else if(value.length==20){
                _vauleBuf.clear();
                value.forEach((v){
                  _vauleBuf.add(v);
                });
                //_vauleBuf.addAll(value);
                print("a half!");
              }else if(value.length ==10){
                //遥控数据
                print("remote: $value");
              }else {
                //用于错位后对齐
                print(value.length);
              }
              //print(value);
            });
            _timerShowIco.cancel();
            _getJKList= Timer.periodic(Duration( milliseconds: 500), (timer){
              List<int> sendData=List(10);
              sendData[0]=0x00;
              sendData[1]=0x34;
              sendData[2]=0x35;
              sendData[3]=0x01;
              var numMS=(DateTime.now().millisecondsSinceEpoch~/1000);
              sendData[4]=numMS%0xff;
              sendData[5]=(numMS>>8)%0xff;
              sendData[6]=(numMS>>16)%0xff;
              sendData[7]=(numMS>>24)%0xff;
              sendData[8]=sendData[1]^sendData[2]^sendData[3]^sendData[4]^sendData[5]^sendData[6]^sendData[7];
              sendData[9]=0;
              _BTCharacter.write(sendData,withoutResponse: true);
            });
            setState(() {
              wifiColor=Colors.black;
            });
          }
        });
      }
    });
  }

  void blueToothConect() async {
    _BTIndi = FlutterBlue.instance;
    var scanSubscription;
    scanSubscription = _BTIndi.scan().listen((scanResult) {
      // do something with scan result
      print("has find device ${scanResult.device.name} ,and uuid is ${scanResult.device.id}");
      if (scanResult.device.id.toString() == _BTid) {
        scanSubscription.cancel();
        _BTIndi.stopScan();
        print("has find the BB5;");
        _BTDevice = scanResult.device;
        _buleTootchGetChar();
      }
      //print(scanResult);
      //print(scanResult.device.name);
    });
    print("run to conect device!");
  }

  void updateRc(int ctrl){
    List<int> sendData=List(10);
    sendData[0]=0x00;
    sendData[1]=0x34;
    sendData[2]=0x35;
    sendData[3]=0x00;
    sendData[4]=fwValue +((waterTemplate.toInt()-30)<<4);
    sendData[5]=(pjPos.toInt()<<4)+ fsfPos.toInt();
    sendData[6]=(zwValue<<4)+(message?8:0) +(hotWaterOn?4:0)+(ledOn?2:0);
    sendData[7]=ctrl;
    sendData[8]=sendData[1]^sendData[2]^sendData[3]^sendData[4]^sendData[5]^sendData[6]^sendData[7];
    sendData[9]=0x00;
    _BTCharacter.write(sendData,withoutResponse: true);
  }

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    var titleStyle=TextStyle(fontSize: 20,color: Colors.black54,fontWeight:FontWeight.bold  );
    var numStyle=TextStyle(fontSize: 25,color: Colors.indigo);
    return Scaffold(
      appBar: new AppBar(title: Text("马桶控制面板")),
      body: Padding(
        padding: EdgeInsets.all(3),
        child: Column(
          children: <Widget>[
            Container(
              //padding: EdgeInsets.all(5),
              width: 360,
              height: 120,
              decoration: BoxDecoration(
                  border: Border.all(color: Colors.teal, width: 2)),
              child: Row(
                children: <Widget>[
                  Container(
                    width: 100,
                    child: Column(children: <Widget>[
                      Row(
                        children: <Widget>[
                          Text("水温:",style: titleStyle,),
                          Text(waterTemplate.toInt().toString(),style: numStyle,),
                        ],
                      ),
                      Row(
                        children: <Widget>[
                          Text("风温:",style: titleStyle,),
                          fwIco
                        ],
                      ),Row(
                        children: <Widget>[
                          Text("座温:",style: titleStyle,),
                          Text(zwValue.toString(),style: numStyle,)
                        ],
                      )
                    ]),
                  ),
                  Expanded(
                    child: Column(
                      children: <Widget>[
                        Row(
                          children: <Widget>[
                            Text("喷头位置:",style: titleStyle,),
                            Text(pjPos.toInt().toString(),style: numStyle,),
                            Text("    水势大小:",style: titleStyle,),
                            Text(fsfPos.toInt().toString(),style: numStyle,)
                          ],
                        ),
                        Table(
                          children: [
                            TableRow(children: [
                              TableCell(child: Icon(Icons.import_export,color: message?Colors.blue:Colors.white,)),
                              TableCell(child: Icon(Icons.bluetooth_audio,color: wifiColor,))
                            ])
                          ],
                        ),
                      ],
                    ),
                  )
                ],
              ),
            ),
            Padding(
              //alignment: Alignment.center,
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Text("水温设置: "),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Text("-"),
                    color: Colors.lightGreen,
                    onPressed: () {
                      if (waterTemplate > 30) {
                        updateRc(0);
                        setState(() {
                          waterTemplate--;
                        });
                      }
                    },
                  ),
                  Text(" "),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    color: Colors.lightGreen,
                    child: Text("+"),
                    onPressed: () {
                      if (waterTemplate < 40) {
                        updateRc(0);
                        setState(() {
                          waterTemplate++;
                        });
                      }
                    },
                  ),
                  CupertinoSlider(
                    value: waterTemplate,
                    //max: 1.0,
                    divisions: 10,
                    min: 30.0,
                    max: 40,
                    onChanged: (double v) {
                      setState(() {
                        waterTemplate = v;
                      });
                    },
                    onChangeEnd: (double v) {
                      updateRc(0);
                    },
                  )
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Text("喷头位置: "),
                  CupertinoButton(
                    padding: EdgeInsets.all(2),
                    child: Text("-"),
                    color: Colors.lightGreen,
                    onPressed: () {
                      if (pjPos > 0) {
                        setState(() {
                          pjPos--;
                        });
                        updateRc(0);
                      }
                    },
                  ),
                  Text(" "),
                  CupertinoButton(
                    padding: EdgeInsets.all(2),
                    color: Colors.lightGreen,
                    child: Text("+"),
                    onPressed: () {
                      if (pjPos < 14) {
                        setState(() {
                          pjPos++;
                        });
                        updateRc(0);
                      }
                    },
                  ),
                  CupertinoSlider(
                    value: pjPos,
                    min: 0,
                    max: 15,
                    divisions: 15,
                    onChanged: (double v) {
                      setState(() {
                        pjPos = v;
                      });
                    },
                    onChangeEnd: (double v){
                      updateRc(0);
                    },
                  ),
                ],
              ),
            ),
            Padding(
              //alignment: Alignment.center,
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Text("水势调节: "),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Text("-"),
                    color: Colors.lightGreen,
                    onPressed: () {
                      if (fsfPos > 0) {
                        setState(() {
                          fsfPos--;
                        });
                        updateRc(0);
                      }
                    },
                  ),
                  Text(" "),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    color: Colors.lightGreen,
                    child: Text("+"),
                    onPressed: () {
                      if (fsfPos < 9) {
                        setState(() {
                          fsfPos++;
                        });
                        updateRc(0);
                      }
                    },
                  ),
                  CupertinoSlider(
                    value: fsfPos,
                    min: 0,
                    max: 9,
                    divisions: 9,
                    onChanged: (double v) {
                      setState(() {
                        fsfPos = v;
                      });
                    },
                    onChangeEnd: (double v){
                      updateRc(0);
                    },
                  ),
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    color: Colors.blue,
                    child: Text("女洗"),
                    onPressed: () {
                      updateRc(0x32);
                    },
                  ),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    //minSize: 100,
                    color: Colors.blue,
                    child: Text(
                      "臀洗",
                    ),
                    onPressed: () {
                      updateRc(0x33);
                    },
                  ),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.wb_sunny, color: ledColor),
                    color: Colors.blue,
                    onPressed: () {
                      setState(() {
                        ledOn = !ledOn;
                        if (ledOn)
                          ledColor = Colors.red;
                        else
                          ledColor = Colors.white;
                      });
                      updateRc(0);
                    },
                  )
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Text("烘干"),
                    color: Colors.blue,
                    onPressed: () {
                      updateRc(0x39);
                    },
                  ),
                  CupertinoButton(
                      padding: EdgeInsets.all(3),
                      child: Icon(Icons.brightness_1),
                      color: Colors.blue,
                      onPressed: () {
                        updateRc(0x56);
                      }),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    color: Colors.blue,
                    child: Icon(Icons.hot_tub,color: hotWater),
                    onPressed: () {
                      setState(() {
                          if(hotWater==Colors.white){
                            hotWater=Colors.red;
                            hotWaterOn=true;
                          }else{
                            hotWater=Colors.white;
                            hotWaterOn=false;
                          }
                      });
                      updateRc(0);
                    },
                  )
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.stop),
                    color: Colors.blue,
                    onPressed: () {
                      updateRc(0xff);
                    },
                  ),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.brightness_3),
                    color: Colors.blue,
                    onPressed: () {
                      updateRc(0x45);
                    },
                  ),
                  CupertinoButton(
                    color: Colors.blue,
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.import_export,color: message?Colors.red:Colors.white,),
                    onPressed: (){
                      setState(() {
                        message=!message;
                      });
                      updateRc(0);
                    },
                  )
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  CupertinoButton(
                    child: Text("风温-"),
                    padding: EdgeInsets.all(5),
                    color: Colors.blue,
                    onPressed: () {
                      if(fwValue>0){

                        setState(() {
                          fwValue--;

                          switch (fwValue) {
                            case 0:
                              fwIco = Icon(Icons.cancel);
                              break;
                            case 1:
                              fwIco = Icon(Icons.filter_1);
                              break;
                            case 2:
                              fwIco = Icon(Icons.filter_2);
                              break;
                            case 3:
                              fwIco = Icon(Icons.filter_3);
                              break;
                            case 4:
                              fwIco = Icon(Icons.hot_tub);
                              break;
                            default:
                              fwIco = Icon(Icons.cancel);
                          }
                        });
                        updateRc(0);
                      }},
                  ),
                  CupertinoButton(
                    child: Text("风温+"),
                    padding: EdgeInsets.all(5),
                    color: Colors.blue,
                    onPressed: () {
                      if(fwValue<4){
                        setState(() {
                          fwValue++;
                          switch(fwValue){
                            case 0:
                              fwIco=Icon(Icons.cancel) ;
                              break;
                            case 1:
                              fwIco=Icon(Icons.filter_1);
                              break;
                            case 2:
                              fwIco=Icon(Icons.filter_2);
                              break;
                            case 3:
                              fwIco =Icon(Icons.filter_3);
                              break;
                            case 4:
                              fwIco =Icon(Icons.hot_tub);
                              break;
                            default:
                              fwIco =Icon(Icons.cancel);
                          }

                        });
                        updateRc(0);
                      }
                    },
                  ),
                  CupertinoButton(
                    padding: EdgeInsets.all(3),
                    child: Text("座温+"),
                    color: Colors.blue,
                    onPressed: () {
                      if(zwValue<4){
                        setState(() {
                          zwValue++;
                        });
                        updateRc(0);
                      }
                    },
                  ),

                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(5.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  CupertinoButton(
                    color: Colors.blue,
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.assistant),
                    onPressed: (){
                      Navigator.of(context).pushNamed("JKList");
                    },
                  ),
                  CupertinoButton(
                    color: Colors.blue,
                    padding: EdgeInsets.all(3),
                    child: Icon(Icons.refresh),
                    onPressed: (){
                      //loop
                      //if(_timerShowIco.isActive) return;
                      List<int> sendData=List(10);
                      sendData[0]=0x00;
                      sendData[1]=0x34;
                      sendData[2]=0x35;
                      sendData[3]=0x01;
                      var numMS=(DateTime.now().millisecondsSinceEpoch~/1000);
                      sendData[4]=numMS%0xff;
                      sendData[5]=(numMS>>8)%0xff;
                      sendData[6]=(numMS>>16)%0xff;
                      sendData[7]=(numMS>>24)%0xff;
                      sendData[8]=sendData[1]^sendData[2]^sendData[3]^sendData[4]^sendData[5]^sendData[6]^sendData[7];
                      sendData[9]=0;
                      _BTCharacter.write(sendData,withoutResponse: true);
                      //end
                    },
                  ),
                  CupertinoButton(
                    child: Text("座温-"),
                    padding: EdgeInsets.all(5),
                    color: Colors.blue,
                    onPressed: () {
                      if(zwValue>0) {
                        setState(() {
                          zwValue--;
                        });
                        updateRc(0);
                      }
                    },
                  )

                ],
              ),
            )
          ],
        ),
      ),
    );
  }


}
