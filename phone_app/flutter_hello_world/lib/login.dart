import "package:flutter/widgets.dart";
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:shared_preferences/shared_preferences.dart';

class login extends StatefulWidget {
  final title = "登陆";

  @override
  LoginData createState() {
    // TODO: implement createState
    return LoginData();
  }
}

class LoginData extends State<login> {
  String name = "";
  var password = "";
  FocusNode userFN = FocusNode();
  FocusNode passFN = FocusNode();

  Future _hasLogin() async{
    var spspone=await SharedPreferences.getInstance();

    spspone.setString("has_login", "admin");
  }

  @override
  void initState() {
    super.initState();

  }

  Future<void> _showAlert() async {
    return showDialog<void>(
        context: context,
        barrierDismissible: false,
        builder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: Text('密码错误'),
            content: Text('重新确认账号密码'),
            actions: <Widget>[
              new CupertinoButton(onPressed: () {
                Navigator.of(context).pop();
              }, child: Text('确认')),
            ],
          );
        }
    );
  }

  Widget build(BuildContext context) {
    print(MediaQuery.of(context).size);
    // TODO: implement build
    return Scaffold(
        appBar: AppBar(title: Text(widget.title)),
        body: Center(
          child: Column(
            mainAxisSize: MainAxisSize.max,
            children: [
              Row(
                children: <Widget>[
                  //Image(

                  //),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  Center(
                    widthFactor: 2.0,
                    child: Text(
                      '账号:',
                      textScaleFactor: 1.2,
                    ),
                  ),
                  Expanded(
                    child: TextField(
                      focusNode: userFN,
                      decoration: InputDecoration(
                          hintText: "邮箱/手机号/账号",
                          prefixIcon: Icon(Icons.account_box),
                          border: OutlineInputBorder()),
                      onChanged: (input) {
                        this.name = input;
                      },
                      onEditingComplete: () {
                        FocusScope.of(context).requestFocus(passFN);
                      },
                      autofocus: true,
                    ),
                  )
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  Center(
                    widthFactor: 2.0,
                    child: Text(
                      "密码: ",
                      textScaleFactor: 1.2,
                      style:
                          TextStyle(textBaseline: TextBaseline.alphabetic),
                    ),
                  ),
                  Expanded(
                    child: TextField(
                      focusNode: passFN,
                      obscureText: true,
                      maxLength: 9,
                      decoration: InputDecoration(
                          hintText: "5~9位密码",
                          border: OutlineInputBorder(),
                          prefixIcon: Icon(Icons.vpn_key)),
                      onChanged: (String input){
                        this.password=input;
                      },
                    ),
                  )
                ],
              ),
              ButtonBar(
                  children: <Widget>[
                FlatButton(
                  onPressed: () {
                    if(password=="admin" && name == "admin"){
                      _hasLogin();

                      Navigator.of(context).pushNamed("conBan");
                    }
                    else _showAlert();
                  },
                  color: Colors.blueAccent,
                  textColor: Colors.white,
                  child: Text("登陆"),
                ),
                RaisedButton(
                  color: Colors.pinkAccent,
                  textColor: Colors.white,
                  child: Text("注册新账号"),
                  onPressed: () {},
                )
              ]),
            ],
          ),
        ));
  }
}
