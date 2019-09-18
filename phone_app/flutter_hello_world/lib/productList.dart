import 'package:flutter/material.dart';

class ProductLiestView extends StatefulWidget{
  @override
  ProductListState createState() {
    // TODO: implement createState
    return ProductListState();
  }
}

class ProductListState extends State<ProductLiestView>{
  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: Text("已经对接的马桶设备"),
        
      ),
      body: Center(
        child: ListView(
          children: <Widget>[
            Container(
              height: 100,
              child: Text("haha"),
              decoration: BoxDecoration(
                color: Colors.blue
              ),
            ),
            Container(
              height: 100,
              decoration: BoxDecoration(
                color: Colors.red
              ),
              child: Text("haha"),
            ),

          ],
        ),
      ),
      floatingActionButton: 
      FloatingActionButton(
        child: Icon(Icons.add),
          onPressed: (){
        
      })
      ,
    );
  }
}