//温湿度センサモジュールを使って温度、湿度を測定するスケッチ
#include "Arduino.h"
#include "SPI.h"
#include "DHT.h"

#define DHT_Pin	(2)		//DHT11のDATAピンを定義
#define DHT_Type	(DHT11) //センサの型番定義　DHT11,DHT22など

DHT dht(DHT_Pin, DHT_Type); //センサ初期化

//計測値
float humidity = 0.0f; //湿度
float temperature = 0.0f; 	//温度

void setup() {
	Serial.begin(9600);
	Serial.println("DHT11 sample"); //画面に表示
	dht.begin(); //温湿度センサー開始
}

void loop() {
	delay(5000); //正確なデータ取得のために5秒以上間隔を置く

	//データ読み出し
	humidity = dht.readHumidity(); //湿度の読み出し
	temperature = dht.readTemperature(); //温度の読み出し(摂氏)

	//読み取り値の確認
	if (isnan(humidity) || isnan(temperature)) {
		Serial.println("Read failure!");
		return;
	}

	/* 以下読み取り値の表示 */
	Serial.print("湿度: ");
	Serial.print(humidity);
	Serial.println("[%]");
	Serial.print("温度: ");
	Serial.print(temperature);
	Serial.println("[℃]");
}
