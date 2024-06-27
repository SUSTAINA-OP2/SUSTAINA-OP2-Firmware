# 校正方法
## build方法
- `cmake -Bbuild -GNinja`
- `cd build`
- `ninja`  
`calibrate_foot_sensor_gui`が生成される

## 実行
`./calibrate_foot_sensor_gui` を実行すると以下のGUIが表示される  
![alt text](image.png)
左上`init`ボタンをクリックするとセンサの読み取りを開始する

- TargetFoot: 右足か左足か選択する
- TargetSensorPosition: 画像の通りの配置
- weight: 校正するために乗せる重りの重さ単位[g]
- times: 何回の平均を取るか
- calibrate ボタン：weight, timesでセンサ値を実際に取得
    校正するためのデータに使用
- offset: ゼロ点を取るために何回の平均にするか
- 右側の`Start Calculation`
    - 取ったセンサ値でキャリブレーションを実行するボタン
    - TargetFootで選択されている足の計算をする

## 流れ
- 重りの準備
- ロボットを仰向けで寝かせる
    - 校正するときは足裏が地面と平行になるように
- プログラムの実行
    - `./calibrate_foot_sensor_gui`
- サーボの電源を入れる
- GUIの`init`ボタンを押す
- TargetFoot, TargetSensorPositionで選択した場所に重りを乗せる
    - 重りは500, 200, 100, 50 [g]で行う 
        - 重りを乗せる
        - `calibrate`ボタンを押す
        - 上記を重り分繰り返し片足４カ所やる

- ４カ所の校正が終わったら`StartCalcurationボタンを押す
- [right or left]_foot_sensor_bias.ymlファイルができる  
下に表示されているセンサ値も校正したscale値が反映される  
ゼロ点補正をするとより正確に表示される
    何も乗せない状態で`offset`ボタンを押す


