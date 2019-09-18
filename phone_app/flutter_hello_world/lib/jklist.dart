import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:sqflite/sqflite.dart';

class JkListWidgets extends StatefulWidget {
  @override
  JkListState createState() {
    // TODO: implement createState
    return JkListState();
  }
}

class JkListState extends State<JkListWidgets> {
  List bodyList = List();
  Database jkl;
  var databasePath;

  void loadData() async {
    if (jkl == null) return;
    bodyList.clear();
    jkl.query('jkList').then((v1) {
      v1.forEach((value) {
        bodyList.add(value);
      });
      setState(() {
      });
    });
    //jkl.close();
  }

  Future<void> openDB() async {
    var databasePath = await getDatabasesPath();
    String path = databasePath + "jk";
    jkl = await openDatabase(path, version: 1,
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
      //await db.close();
    });
    loadData();
  }

  @override
  void initState() {
    // TODO: implement initState
    openDB();
    loadData();
    print("init");
    super.initState();
  }

  @override
  void dispose() {
    // TODO: implement dispose
    jkl.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: Text("马桶数据记录"),
      ),
      body:  ListView.builder(
        itemCount: bodyList.length,
          itemBuilder: (context,id){
            var iwt=bodyList[id];
            //print(id);
              return ListTile(
                title: Text(
                    "于${DateTime.fromMillisecondsSinceEpoch(iwt['tick'] * 1000+iwt['sit_tick']).toIso8601String()}"),
                leading:
                Icon(iwt['do'] == 3 ? Icons.pregnant_woman : Icons.autorenew),
                subtitle: Text(
                    "总用时:${(iwt['exit_tick'] - iwt['sit_tick']) / 1000},蹲坐时间:${(iwt['do_tick'] - iwt['sit_tick']) / 1000}"),
              );
          },
          ),
      endDrawer: Drawer(
        child: Column(
          children: <Widget>[
            DrawerHeader(
                child: ListTile(
              title: Text("用户"),
              leading: Icon(Icons.account_circle),
              subtitle: Text("健康数据中心"),
            )),
            ListTile(
              leading: Icon(Icons.settings),
              title: Text('更新信息'),
              onTap: () {
                jkl.insert("jkList", {
                  "tick":
                      (DateTime.now().millisecondsSinceEpoch ~/ 1000) - 4000,
                  "do": 5,
                  "do_tick": 45,
                  "hg_tick": 2000,
                  "sit_tick": 5,
                  "exit_tick": 5000
                }).catchError((ojb) {
                  print("some error!");
                });
              },
            ),
            ListTile(
              leading: Icon(Icons.delete),
              title: Text('删除数据'),
              onTap: () {
                jkl.delete("jkList");
              },
            ),
            ListTile(
                leading: Icon(Icons.refresh),
                title: Text("刷新数据"),
                onTap: () {
                  loadData();
                }),
            ListTile(
              leading: Icon(
                Icons.alternate_email,
              ),
              title: Text("谢谢试用本demo!"),
            )
          ],
        ),
      ),
    );
  }
}
