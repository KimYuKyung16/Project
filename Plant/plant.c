#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <softPwm.h>
#include <softTone.h>
#include <lcd.h>
#include <stdlib.h>
#include <stdint.h>

//keypad (11개)
#define PW1 2
#define PW2 3
#define PW3 4
#define PW4 17
#define PW5 27 
#define PW6 22
#define PW7 5
#define PW8 6
#define PW9 13
#define PW0 26 
#define PW_S 14 //스크롤 겸 세팅 버튼

//buzzer
#define BUZZER_PIN 5 

//freq
#define DO_L 523
#define RE 587
#define MI 659
#define FA 698
#define SOL 784
#define RA 880
#define SI 987
#define DO_H 1046
#define RE_H 1174
#define MI_H 1318
#define FA_H 1396
#define SOL_H 1568

#define MAX_KEY_BT_NUM 10 //키패드 0~9까지 총 10개를 뜻한다.

#define A_1A  15 //water pump  
#define A_2A  18 //water pump

#define MT_P 7 //dc motor
#define MT_N 19 //dc motor

//dc motor control (방향)
#define LEFT_ROTATE 1 
#define RIGHT_ROTATE 2

//sensor SPI
#define CS_MCP3208 8 
#define SPI_CHANNEL 0 
#define SPI_SPEED 1000000 

//DHT
#define MAXTIMINGS 83
#define DHTPIN 25
int dht11_dat[5] = { 0, };

const int KeypadTable[MAX_KEY_BT_NUM] = { PW0, PW1,PW2,PW3,PW4,PW5,PW6,PW7,PW8,PW9 }; //키패드 배열에 0~9까지 저장

int set_tem, set_shum, set_water = 0; //온도, 토양습도, 물양 설정값 변수
int tem, hum = 0; //식물 주변 온도와 습도 값 변수
int plant_state = 0; //식물의 현재 상태 변수
int water_result, shum_result = 0; //워터펌프 딜레이값 계산 변수와 토양습도 센서값을 %로 바꾼 값 변수 
int mcount = 2; //모터 돌리는 방향 설정할 때 필요한 변수

int tem2[2], hum2[2], water3[3]; //사용자로부터 입력받은 온도, 토양습도, 물양 변수

void FanOn(void); //water pump 작동 함수
void FanOff(void); //water pump 멈춤 함수


//DC모터 제어 함수들
void MotorStop() //모터를 멈추는 함수
{
	softPwmWrite(MT_P, 0);
	softPwmWrite(MT_N, 0);
}
void MotorControl(unsigned char speed, unsigned char rotate) //모터를 돌리는 함수
{
	if (rotate == LEFT_ROTATE) { //모터 왼쪽으로 회전
		digitalWrite(MT_P, LOW);
		softPwmWrite(MT_N, speed);
	}
	else if (rotate == RIGHT_ROTATE) { //모터 오른쪽으로 회전
		digitalWrite(MT_N, LOW);
		softPwmWrite(MT_P, speed);
	}
}

//Buzzer 관련 함수들
unsigned int SevenScale(unsigned char scale) //원하는 음계 선택 가능
{
	unsigned int _ret = 0;
	switch (scale)
	{
	case 0:
		_ret = DO_L;
		break;
	case 1:
		_ret = RE;
		break;
	case 2:
		_ret = MI;
		break;
	case 3:
		_ret = FA;
		break;
	case 4:
		_ret = SOL;
		break;
	case 5:
		_ret = RA;
		break;
	case 6:
		_ret = SI;
		break;
	case 7:
		_ret = DO_H;
		break;
	case 8:
		_ret = RE_H;
		break;
	case 9:
		_ret = MI_H;
		break;
	case 10:
		_ret = FA_H;
		break;
	case 11:
		_ret = SOL_H;
		break;
	}
	return _ret;
}

void Change_FREQ(unsigned int freq) //소리를 바꾸는 함수
{
	softToneWrite(BUZZER_PIN, freq);
}

void STOP_FREQ(void) //소리를 멈추는 함수
{
	softToneWrite(BUZZER_PIN, 0);
}

void Buzzer_Init(void) //Buzzer 정의
{
	softToneCreate(BUZZER_PIN);
	STOP_FREQ();
}


//ADC 함수 --- 센서 3개를 제어
int ReadMcp3208ADC(unsigned char adcChannel)
{
	unsigned char buff[3];
	int nAdcValue = 0;
	buff[0] = 0x06 | ((adcChannel & 0x07) >> 2);
	buff[1] = ((adcChannel & 0x07) << 6);
	buff[2] = 0x00;
	digitalWrite(CS_MCP3208, 0);
	wiringPiSPIDataRW(SPI_CHANNEL, buff, 3);
	buff[1] = 0x0F & buff[1];
	nAdcValue = (buff[1] << 8) | buff[2];
	digitalWrite(CS_MCP3208, 1);
	return nAdcValue;
}

//온습도를 받아오는 함수
void read_dht11_dat()
{
	uint8_t laststate = HIGH;
	uint8_t counter = 0;
	uint8_t j = 0, i;
	dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;
	pinMode(DHTPIN, OUTPUT);
	digitalWrite(DHTPIN, LOW);
	delay(18);
	digitalWrite(DHTPIN, HIGH);
	delayMicroseconds(30);
	pinMode(DHTPIN, INPUT);

	for (i = 0; i < MAXTIMINGS; i++) {
		counter = 0;
		while (digitalRead(DHTPIN) == laststate) {
			counter++;
			delayMicroseconds(1);
			if (counter == 200) break;
		}
		laststate = digitalRead(DHTPIN);
		if (counter == 200) break;
		if ((i >= 4) && (i % 2 == 0)) {
			dht11_dat[j / 8] <<= 1;
			if (counter > 20) dht11_dat[j / 8] |= 1;
			j++;
		}
	}
	if ((j >= 40) && (dht11_dat[4] == ((dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xff))) {
		tem = dht11_dat[0];
		hum = dht11_dat[2];
	}
	else printf("Data get failed\n");
}



int main(void) {
	int nCdsChannel = 0; //조도센서의 채널은 0으로 설정
	int soilChannel = 1; //토양습도센서의 채널은 1로 설정
	int moistChannel = 2; //수위센서의 채널은 2로 설정

	int nCdsValue = 0; //조도 초기값
	int soilValue = 0; //토양습도 초기값
	int moistValue = 0; //수위센서 초기값

	int disp1; //LCD 변수
	int S_count = 0, S_count4 = 0; //메뉴 4개일 때의 count변수, 메뉴 3개일 때의 count변수
	//메뉴1번 누를때 상태값 변수, 메뉴2번 누를때 상태값 변수, 메뉴3번 누를때 상태값 변수, 메뉴4번 누를때 상태값 변수
	int count1 = 0, count2 = 0, count3 = 0, count4 = 0; 
	int count = 0; //온도, 토양습도, 물양 자리수 저장 변수
	int setting = 0; //현재 메뉴 상태값 변수


	if (wiringPiSetupGpio() == -1) {
		fprintf(stdout, "Not start wiringPi: %s\n", strerror(errno));
		return 1;
	}

	if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
		fprintf(stdout, "wiringPiSPISetup Failed: %s\n", strerror(errno));
		return 1;
	}

	//DC모터 OUTPUT 설정 및 생성
	pinMode(MT_P, OUTPUT);
	pinMode(MT_N, OUTPUT);
	softPwmCreate(MT_P, 0, 100);
	softPwmCreate(MT_N, 0, 100);

	//워터펌프 OUTPUT 설정
	pinMode(A_1A, OUTPUT);
	pinMode(A_2A, OUTPUT);

	//ADC OUTPUT 설정
	pinMode(CS_MCP3208, OUTPUT);

	//키패드 INPUT 설정
	for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
		pinMode(KeypadTable[i], INPUT);
	}

	disp1 = lcdInit(2, 16, 4, 23, 24, 20, 21, 12, 16, 0, 0, 0, 0); //LCD 초기화
	lcdClear(disp1); //LCD 화면 지우기

	Buzzer_Init(); //부저 초기화
	read_dht11_dat(); //온습도값 받아오기

	lcdClear(disp1); //LCD 화면 지우기
	lcdPosition(disp1, 0, 0);
	lcdPrintf(disp1, "Tem:%d", tem); //첫 번째줄에 센서로 받아온 온도값 출력 
	lcdPosition(disp1, 7, 0);
	lcdPrintf(disp1, "Hum:%d", hum); //첫 번째줄에 센서로 받아온 습도값 출력
	lcdPosition(disp1, 0, 1);
	lcdPrintf(disp1, "PlantState:%d", plant_state); //두 번째줄에 선택된 식물 상태가 무엇인지 출력
	lcdPosition(disp1, 15, 1);
	lcdPrintf(disp1, "S"); //두 번째줄에 세팅을 나타내는 S 출력

	while (1) {
		MotorStop(); //DC모터는 항상 멈춘 상태에서 시작
		FanOff(); //워터펌프는 항상 멈춘 상태에서 시작

		nCdsValue = ReadMcp3208ADC(nCdsChannel); //센서에서 조도값을 받아와서 ADC로 변환 후 값 저장 
		soilValue = ReadMcp3208ADC(soilChannel); //센서에서 토양습도값을 받아와서 ADC로 변환 후 값 저장 
		moistValue = ReadMcp3208ADC(moistChannel); //센서에서 수위값을 받아와서 ADC로 변환 후 값 저장 

		soilValue = (soilValue / 100)*2.5; //센서로 받아온 토양습도값을 %로 나타내기 위한 계산식 

		printf("Soil Sensor Value = % u\n", soilValue); //토양습도 값 화면으로 출력
		printf("Moist Sensor Value = % u\n", moistValue); //수위값 화면으로 출력
		printf("Cds : %u \n", nCdsValue); //조도값 화면으로 출력

		read_dht11_dat(); //온습도를 계속해서 업데이트하기 위해 받아오기

		//LCD 초기 화면
		lcdPosition(disp1, 0, 0);
		lcdPrintf(disp1, "Tem:%d", tem); //첫 번째줄에 온도 출력
		lcdPosition(disp1, 7, 0);
		lcdPrintf(disp1, "Hum:%d", hum); //첫 번째줄에 습도 출력
		lcdPosition(disp1, 0, 1);
		lcdPrintf(disp1, "PlantState:%d", plant_state); //두 번째줄에 설정된 식물상태 출력
		lcdPosition(disp1, 15, 1);
		lcdPrintf(disp1, "S"); //두 번째줄에 세팅을 나타내는 S 출력

		//블라인드를 컨트롤 하는 IF문
		if (set_tem != 0) { //블라인드 컨트롤을 위한 온도값이 설정되어 있을 때
			/*블라인드 내리기
			조도값이 3000이상이고 현재 온도가 사용자가 설정한 온도보다 높을 때, mcount가 2일 때
			mcount가 1인지 2인지를 설정함으로써 내려진 블라인드는 더이상 내리는 동작을 할 수 없고 올리는 동작만 가능해진다.*/
			if (nCdsValue >= 3000 && tem >= set_tem && mcount == 2) { 
				MotorStop(); //모터 멈추기
				delay(2000); //멈춘 상태를 2초정도 유지
				MotorControl(50, LEFT_ROTATE); //모터를 왼쪽으로 회전함으로써 블라인드가 내려진다.
				delay(9000);//9초 동안 블라인드를 내린다.
				MotorStop(); //9초 후 블라인드를 멈춘다.
				mcount = 1; //다음 동작은 블라인드 올리는 동작이 나오도록 mcount를 1로 설정
			}
			/*블라인드 올리기
			조도값이 3000미만이고 현재 온도가 사용자가 설정한 온도보다 낮을 때, mcount가 1일 때
			mcount가 1인지 2인지를 설정함으로써 올려진 블라인드는 더이상 올리는 동작을 할 수 없고 내리는 동작만 가능해진다.*/
			if (nCdsValue < 3000 && tem < set_tem && mcount == 1) { 
				MotorStop(); //모터 멈추기
				delay(2000); //멈춘 상태를 2초정도 유지
				MotorControl(50, RIGHT_ROTATE); //모터를 왼쪽으로 회전함으로써 블라인드가 올라간다.
				delay(9000); //9초 동안 블라인드를 내려졌기 때문에 올릴 때도 9초를 준다.
				MotorStop(); //9초 후 블라인드를 멈춘다.
				mcount = 2; //다음 동작은 블라인드 내리는 동작이 나오도록 mcount를 2로 설정
			}
		}

		//토양습도가 낮을 때 워터펌프를 동작하는 부분
		if (set_shum != 0) { //워터펌프를 동작시키기 위한 토양습도값이 설정되어있을 때
			if (soilValue < set_shum) { //토양습도값이 사용자가 설정한 토양습도 값보다 낮을 경우
				lcdClear(disp1); //LCD 지우기
				lcdPosition(disp1, 0, 0);
				delay(2000);
				lcdPrintf(disp1, "Watering %dml", set_water); //LCD에 사용자가 설정해놓은 물양값만큼 물을 준다는 화면이 나온다.
				lcdPosition(disp1, 0, 1);
				lcdPuts(disp1, "Please Wait..."); 

				FanOn(); //워터펌프를 동작시킨다.
				delay(water_result); //사용자가 설정한 물양값을 계산해서 워터펌프 딜레이값 만큼 딜레이를 주어서 물양을 조절한다.
				lcdClear(disp1);
			}
		}	

		//수위값이 낮으면 부저를 울린다. 
		if (moistValue < 1500){ //수위값이 1500미만일 경우
			lcdClear(disp1); 
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Water shortage"); //LCD에 물부족을 알려주는 알림을 띄우고 물을 넣으라고 알려준다.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Put Water...");

			//물 부족 경고음을 울린다.
			Change_FREQ(SevenScale(5));
			delay(300);
			STOP_FREQ();
			Change_FREQ(SevenScale(1));
			delay(300);
			STOP_FREQ();
			delay(1000);
		}

		//자동으로 물을 주고싶을 때 말고 수동으로 물을 주고싶을 때 
		//수동으로 물주기 1번(적은 물 양)
		if (!digitalRead(PW1)) { //1번 키패드를 눌렀을 때
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 50ml"); //LCD에 50ml의 물을 준다는 화면을 띄운다.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD에 기다리라고 띄운다..

			FanOn(); //워터 펌프를 동작시킨다.
			delay(5000); //워터펌프 1초에 10ml의 물이 나오기 때문에 50ml의 물은 5초의 딜레이를 준다.
			lcdClear(disp1);
		}
		//수동으로 물주기 2번(중간 물 양)
		if (!digitalRead(PW2)) { //2번 키패드를 눌렀을 때
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 100ml"); //LCD에 100ml의 물을 준다는 화면을 띄운다.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD에 기다리라고 띄운다..

			FanOn(); //워터 펌프를 동작시킨다.
			delay(10000); //워터펌프 1초에 10ml의 물이 나오기 때문에 100ml의 물은 10초의 딜레이를 준다.
			lcdClear(disp1);
		}
		//수동으로 물주기 3번(많은 물 양)
		if (!digitalRead(PW3)) { //3번 키패드를 눌렀을 때
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 150ml"); //LCD에 150ml의 물을 준다는 화면을 띄운다.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD에 기다리라고 띄운다..

			FanOn(); //워터 펌프를 동작시킨다.
			delay(15000); //워터펌프 1초에 10ml의 물이 나오기 때문에 150ml의 물은 15초의 딜레이를 준다.
			lcdClear(disp1);
		}

		//식물의 설정 상태 정보를 더 자세하게 알고싶을 때
		if (!digitalRead(PW0)) { //키패드 0번을 눌렀을 때
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();
			setting = 2; //setting을 2로 한다.
			lcdClear(disp1);

			while (setting == 2) { //setting이 2일 경우에 계속 반복
				//lcdClear(disp1);
				lcdPosition(disp1, 0, 0);
				lcdPrintf(disp1, "Tem:%d", set_tem); //설정된 온도 값 출력
				lcdPosition(disp1, 7, 0);
				lcdPrintf(disp1, "SHum:%d", set_shum); //설정된 토양습도 값 출력
				lcdPosition(disp1, 0, 1);
				lcdPrintf(disp1, "Water:%d", set_water); //설정된 물양 값 출력
				lcdPosition(disp1, 15, 1);
				lcdPuts(disp1, "B"); //back할 수 있도록 B를 표시

				if (!digitalRead(PW_S)) { //위에 나온 B에 해당하는 버튼(PW_S)을 눌렀을 때 초기화면으로 돌아가기
					setting = 0; //while문을 빠져나오기 위해 setting값 0
					delay(500); //초기화면의 setting을 의미하는 S가 눌리지않게 하기 위해 delay 0.5초로 버튼눌림을 끊어준다.
					lcdClear(disp1);
					break; //while문 빠져나오기 (초기화면으로 돌아갈 수 있다.)
				}
			}
		}

		//현재 토양습도 값이 알고싶을 때
		if (!digitalRead(PW4)) { //키패드 4번을 눌렀을 때
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			setting = 4; //setting을 4로 한다.
			lcdClear(disp1);

			while (setting == 4) { //setting이 4일 경우에 계속 반복
				//lcdClear(disp1);
				lcdPosition(disp1, 0, 0);
				lcdPrintf(disp1, "Current Value");
				lcdPrintf(disp1, "SoilHum:%d", soilValue); //현재 토양습도 값 출력
				lcdPosition(disp1, 0, 10);
				lcdPrintf(disp1, "%");
				lcdPosition(disp1, 15, 1);
				lcdPuts(disp1, "B"); //back할 수 있도록 B를 표시

				if (!digitalRead(PW_S)) { //위에 나온 B에 해당하는 버튼(PW_S)을 눌렀을 때 초기화면으로 돌아가기
					setting = 0; //while문을 빠져나오기 위해 setting값 0
					delay(500); //초기화면의 setting을 의미하는 S가 눌리지않게 하기 위해 delay 0.5초로 버튼눌림을 끊어준다.
					lcdClear(disp1);
					break; //while문 빠져나오기 (초기화면으로 돌아갈 수 있다.)
				}
			}
		}

		//SET 메뉴로 가기
		if (!digitalRead(PW_S)) { //PW_S 버튼을 누를 때 set 메뉴로 간다.
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			setting = 1; //setting값을 1로 준다.

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "1.Plant1");
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "2.Plant2");
			lcdPosition(disp1, 15, 1);
			lcdPuts(disp1, "D");

			delay(500); //set 메뉴로 갈 때의 S버튼 눌림과 스크롤 S버튼 눌림이 겹치지 않게 delay

			while (setting == 1) { //setting값이 1일 때 반복
				if (!digitalRead(PW_S)) { //S버튼을 눌렀을 때 스크롤
					Change_FREQ(SevenScale(9));
					delay(200);
					STOP_FREQ();

					S_count++; //버튼을 누를 때마다 S_count를 1씩 올린다.
					S_count %= 4; //메뉴가 4개이기 때문에 4로 나눈 나머지를 구한다.

					switch (S_count) { //S_count값 기준
					case 0: //S_count값이 0일 때 메뉴1과 2가 나온다.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "1.Plant1");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "2.Plant2");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 1: //S_count값이 1일 때 메뉴2과 3이 나온다.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "2.Plant2");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "3.Plant3");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 2: //S_count값이 2일 때 메뉴3과 4가 나온다.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "3.Plant3");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "4.User Set");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 3: //S_count값이 3일 때 메뉴4과 1이 나온다.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "4.User Set");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "1.Plant1");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;

					}

				}

				//뒤로가기를 하고싶을 때
				if (!digitalRead(PW0)) { //PW0을 누르면 BACK
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					S_count = 0; //메뉴를 1부터 띄우기 위해 S_count=0으로 초기화

					//메인화면으로 돌아가기 때문에 메인화면 LCD에 출력
					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPrintf(disp1, "Tem:%d", tem); //온도값 출력
					lcdPosition(disp1, 7, 0);
					lcdPrintf(disp1, "Hum:%d", hum); //토양습도값 출력
					lcdPosition(disp1, 0, 1);
					lcdPrintf(disp1, "PlantState:%d", plant_state); //식물상태값 출력
					lcdPosition(disp1, 15, 1);
					lcdPrintf(disp1, "S");

					delay(500); //메인에서 PW0 작업과 겹치지 않게 하기 위해 delay
					break;
				}

				//PW1을 누르면 식물상태1 을 설정한다.
				if (!digitalRead(PW1)) { //1번 키패드가 눌렸을 때
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count1 = 1; //count1을 1로 준다.

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:10"); //LCD에 식물상태1의 온도값 설정 보여주기
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:10"); //LCD에 식물상태1의 토양습도값 설정 보여주기
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:50"); //LCD에 식물상태1의 물양값 설정 보여주기
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD에 식물 상태 설정을 위한 S 보여주기

					while (count1 == 1) { //count1일 때 반복
						if (!digitalRead(PW_S)) { //PW_S를 누르면 식물상태1 설정 완료
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}
							plant_state = 1; //식물 상태를 1로 바꾼다.

							//시연을 위해 값을 극단적으로 주었다.
							set_tem = 10; //온도는 10도로 설정한다.
							set_shum = 10; //토양습도는 10%로 설정한다.
							set_water = 50; //물 양은 50ml로 설정한다.

							//물 양은 1초에 10ml이기 때문에 50ml는 5초이므로 설정 물양값에서 100을 곱하고 펌프 끌어올리는 시간을 고려해서 2초(2000)를 추가한다.
							water_result = set_water * 100 + 2000; //물 양 딜레이값 설정

							//출력 확인 부분
							printf("%d\n", set_tem);
							printf("%d\n", set_shum);
							printf("%d\n", set_water);

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD에 설정이 잘 되었음을 보여준ㄷ.

							S_count = 0; // 메뉴부분이 1부터 보여지게 하기 위해 S_count를 0으로 초기화
							delay(2000);
							lcdClear(disp1);
							setting = 0; //setting을 0으로 초기화
							break; //설정 후 while문을 빠져나간다.

						}

						//뒤로가기
						if (!digitalRead(PW0)) { //PW0을 누르면 뒤로가기를 한다.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //메뉴가 1부터 나오도록 S_count를 0으로 초기화

							//메뉴 부분 미리 LCD에 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1"); 
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while문 빠져 나가기
						}
					}
				}

				//PW2를 누르면 식물상태2 를 설정한다.
				if (!digitalRead(PW2)) { //2번 키패드가 눌렸을 때
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count2 = 1; //count2를 1로 준다

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:20"); //LCD에 식물상태2의 온도값 설정 보여주기
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:50"); //LCD에 식물상태2의 토양습도값 설정 보여주기
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:100"); //LCD에 식물상태2의 물양값 설정 보여주기
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD에 식물 상태 설정을 위한 S 보여주기

					while (count2 == 1) { //count2가 1일 때 반복
						if (!digitalRead(PW_S)) { //PW_S를 누르면 식물상태2 설정 완료
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}

							plant_state = 2; //식물 상태를 2로 바꾼다.

							//시연을 위해 값을 극단적으로 주었다.
							set_tem = 20; //온도는 20도로 설정한다.
							set_shum = 50; //토양습도는 50%로 설정한다.
							set_water = 100; //물 양은 100ml로 설정한다.

							//물 양은 1초에 10ml이기 때문에 50ml는 5초이므로 설정 물양값에서 100을 곱하고 펌프 끌어올리는 시간을 고려해서 2초(2000)를 추가한다.
							water_result = set_water * 100 + 2000; //물 양 딜레이값 설정

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD에 설정이 잘 되었음을 보여준다.

							S_count = 0; // 메뉴부분이 1부터 보여지게 하기 위해 S_count를 0으로 초기화
							delay(2000);
							lcdClear(disp1); 
							setting = 0; //setting을 0으로 초기화
							break; //설정 후 while문을 빠져나간다.
						}

						//뒤로가기
						if (!digitalRead(PW0)) { //PW0을 누르면 뒤로가기를 한다.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //메뉴가 1부터 나오도록 S_count를 0으로 초기화

							//메뉴 부분 미리 LCD에 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while문 빠져 나가기
						}
					}
				}

				//PW3을 누르면 식물상태3 을 설정한다.
				if (!digitalRead(PW3)) {
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count3 = 1; //count3를 1로 준다.

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:40"); //LCD에 식물상태3의 온도값 설정 보여주기
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:80"); //LCD에 식물상태3의 토양습도값 설정 보여주기
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:150"); //LCD에 식물상태3의 물양값 설정 보여주기
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD에 식물 상태 설정을 위한 S 보여주기

					while (count3 == 1) { //count3이 1일 때 반복
						if (!digitalRead(PW_S)) { //PW_S를 누르면 식물상태3 설정 완료
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}
							plant_state = 3; //식물 상태를 3으로 바꾼다.

							//시연을 위해 값을 극단적으로 주었다.
							set_tem = 40; //온도는 40도로 설정한다.
							set_shum = 80; //토양습도는 80%로 설정한다.
							set_water = 150; //물 양은 150ml로 설정한다.

							//물 양은 1초에 10ml이기 때문에 50ml는 5초이므로 설정 물양값에서 100을 곱하고 펌프 끌어올리는 시간을 고려해서 2초(2000)를 추가한다.
							water_result = set_water * 100 + 2000; //물 양 딜레이값 설정

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD에 설정이 잘 되었음을 보여준다.

							S_count = 0; // 메뉴부분이 1부터 보여지게 하기 위해 S_count를 0으로 초기화
							delay(2000);
							lcdClear(disp1);
							setting = 0; //setting을 0으로 초기화
							break; //설정 후 while문을 빠져나간다.
						}

						//뒤로가기
						if (!digitalRead(PW0)) { //PW0을 누르면 뒤로가기를 한다.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //메뉴가 1부터 나오도록 S_count를 0으로 초기화

							//메뉴 부분 미리 LCD에 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while문 빠져 나가기
						}
					}
				}

				//PW4를 누르면 사용자설정을 할 수 있다.
				if ((!digitalRead(PW4))) { //4번 키패드가 눌렸을 때
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count4 = 1; //count4를 1로 준다.

					//사용자 설정의 메뉴를 LCD에 보여준다.
					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "1.Tem Set"); //온도 직접 설정 
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "2.Hum Set"); //토양 습도 직접 설정
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "D"); //스크롤 D

					while (count4 == 1) {
						if (!digitalRead(PW_S)) { //scroll
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count4++; //PW_S를 누를 때마다 S_count4가 증가
							S_count4 %= 3; //메뉴가 3개이기 때문에 3으로 나눈 나머지를 계산

							switch (S_count4) { //S_count4 기준
							case 0: //S_count4가 0일 때 메뉴1,2 보여주기
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "1.Tem Set"); //1번 메뉴: 온도 설정
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "2.Hum Set"); //1번 메뉴: 토양습도 설정
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //스크롤
								break;
							case 1: //S_count4가 1일 때
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "2.Hum Set"); //2번 메뉴: 토양습도 설정
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "3.Water Set"); //3번 메뉴: 물양 설정
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //스크롤
								break;
							case 2: //S_count4가 2일 때
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "3.Water Set"); //3번 메뉴: 물양 설정
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "1.Tem Set"); //1번 메뉴: 온도 설정
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //스크롤
								break;
							}
						}

						if (!digitalRead(PW0)) { //PW0을 누르면 뒤로가기를 한다.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //메뉴가 1부터 나오도록 S_count를 0으로 초기화
							break; //while문 빠져 나가기
						}


						//PW1을 누르면 사용자가 직접 온도를 설정한다.
						if (!digitalRead(PW1)) { //PW1을 누르면 온도 설정
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//온도 입력을 위한 LCD 입력화면 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Put Temperature");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "->");
							lcdPosition(disp1, 6, 1);
							lcdPuts(disp1, "'C");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "S");

							delay(200);

							while (count < 2) { //count가 2미만일 때 반복, count는 온도 자리수
								for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //버튼 채터링 현상을 막기 위해 버튼이 눌리지 않았을 때만 입력이 들어가게 하였다.
											if (((!digitalRead(PW_S)) == LOW)){ //S일 때는 입력이 안되게 하였다.
												tem2[count] = i; //사용자 설정 온도 변수에 버튼 값 저장
												printf("입력중: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->와 'C 사이에 온도가 입력되는 것을 확인할 수 있다.
												lcdPrintf(disp1, "%d", i); //사용자가 입력한 버튼값을 출력(한자리씩)
												count++; //count를 ++해서 자리수 변경
												continue; //반복
											}
										}
									}
									else {
										continue; //반복
									}
								}
							}

							while (count == 2) { //count가 2일 때, count는 온도 자리수 --> 온도는 한자리는 의미가 없다는 생각이 들어 두자리만 들어가도록 설정
								if ((!digitalRead(PW_S)) == HIGH && count == 2) { //S를 누르고 온도가 두자리 수이면 온도 설정
									for (int i = 0; i < 2; i++) {
										printf("%d", tem2[i]);
									}
									plant_state = 4; //식물 상태를 사용자 설정인 4로 바꾼다.

									//온도를 배열로 받았기 때문에 인덱스 0은 십의 자리수로 10을 곱하고 인덱스1은 일의 자리수로 1을 곱해서 이들을 더했다.
									set_tem = tem2[0] * 10 + tem2[1] * 1; 

									printf("\nTEMP가 설정되었습니다.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD에 설정이 완료되었음을 표시

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //반복을 위한 초기화 작업

									//설정 완료 후 메뉴로 돌아가기 때문에 메뉴 미리 출력
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break; //while문 빠져나가기
								}
							}
						}

						//PW2를 누르면 사용자가 직접 토양 습도를 설정한다.
						if (!digitalRead(PW2)) { //PW2를 누르면 토양습도 설정
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//토양습도 입력을 위한 LCD 입력화면 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Put Humidity");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "->");
							lcdPosition(disp1, 6, 1);
							lcdPuts(disp1, "%");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "S");

							delay(200);

							while (count < 2) { //count가 2미만일 때 반복, count는 토양습도 자리수
								for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //버튼 채터링 현상을 막기 위해 버튼이 눌리지 않았을 때만 입력이 들어가게 하였다.
											if (((!digitalRead(PW_S)) == LOW)) { //S일 때는 입력이 안되게 하였다.
												hum2[count] = i; //사용자 설정 토양습도 변수에 버튼 값 저장
												printf("입력중: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->와 % 사이에 토양습도가 입력되는 것을 확인할 수 있다.
												lcdPrintf(disp1, "%d", i); //사용자가 입력한 버튼값을 출력(한자리씩)
												count++; //count를 ++해서 자리수 변경
												continue; //반복
											}
										}
									}
									else {
										continue; //반복
									}
								}
							}

							while (count == 2) { //count가 2일 때, count는 토양습도 자리수 
								if ((!digitalRead(PW_S)) == HIGH && count == 2) { //S를 누르고 토양습도가 두자리 수이면 토양습도 설정
									for (int i = 0; i < 2; i++) {
										printf("%d", hum2[i]);
									}
									plant_state = 4; //식물 상태를 사용자 설정인 4로 바꾼다.

									//토양습도를 배열로 받았기 때문에 인덱스 0은 십의 자리수로 10을 곱하고 인덱스1은 일의 자리수로 1을 곱해서 이들을 더했다.
									set_shum = hum2[0] * 10 + hum2[1] * 1;

									printf("\nHum가 설정되었습니다.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD에 설정이 완료되었음을 표시

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //반복을 위한 초기화 작업

									//설정 완료 후 메뉴로 돌아가기 때문에 메뉴 미리 출력
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break;  //while문 빠져나가기
								}
							}
						}

						//PW3을 누르면 사용자가 직접 물양을 설정한다.
						if (!digitalRead(PW3)) { //PW3을 누르면 물양 설정
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//온도 입력을 위한 LCD 입력화면 출력
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Put Water");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "->");
							lcdPosition(disp1, 7, 1);
							lcdPuts(disp1, "ml");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "S");

							delay(200);

							while (count < 3) { //count가 3미만일 때 반복, count는 물양 자리수 --> 세 자리까지 입력 가능
								if (!digitalRead(PW_S) == HIGH) { // //S를 눌렀을 때 두자리수이거나 세자리수이면 반복을 끝낸다.
									if (count == 2 || count == 3) { 
										break;
									}
								}

								for (int i = 0; i < MAX_KEY_BT_NUM; i++) { //키패드 입력
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //버튼 채터링 현상을 막기 위해 버튼이 눌리지 않았을 때만 입력이 들어가게 하였다.
											if (((!digitalRead(PW_S)) == LOW)) { //S일 때는 입력이 안되게 하였다.
												water3[count] = i; //사용자 설정 물양 변수에 버튼 값 저장
												printf("입력중: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->와 ml 사이에 물양이 입력되는 것을 확인할 수 있다.
												lcdPrintf(disp1, "%d", i); //사용자가 입력한 버튼값을 출력(한자리씩)
												count++; //count를 ++해서 자리수 변경
												continue;  //반복
											}
										}
									}
									else {
										continue;  //반복
									}
								}
							}

							//count가 2이거나 3일 때, count는 물양 자리수 --> 물양은 3자리수까지 들어갈 수 있어야 좋을 것 같아서 이렇게 설정했다.
							while (count == 2 || count == 3) { 
								if ((!digitalRead(PW_S)) == HIGH && (count == 2 || count == 3)) { //S를 누르고 온도가 두자리 수거나 세자리 수이면 물양 설정
									for (int i = 0; i < 3; i++) {
										printf("%d", water3[i]);
									}
									plant_state = 4; //식물 상태를 사용자 설정인 4로 바꾼다.

									//자리수가 몇이냐에 따라 계산식이 달라지기 때문에 count가 2일때와 3일때로 나누었다.
									if (count == 2) { //두자리수일때
										set_water = water3[0] * 10 + water3[1] * 1; //인덱스0은 십의 자리수이므로 10을 곱하고 인덱스1은 일의 자리수이므로 1을 곱한다.
									}
									if (count == 3) { //세자리수일때
										//인덱스0은 백의 자리수이므로 100을 곱하고 인덱스1은 십의 자리수이므로 10을 곱한다. 인덱스2는 일의 자리수이므로 1을 곱한다.
										set_water = water3[0] * 100 + water3[1] * 10 + water3[2] * 1;
									}

									water_result = set_water * 100 + 2000; //물양을 조절하는 방식이 딜레이이기 때문에 물양 딜레이를 계산한다.

									printf("\nWater가 설정되었습니다.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD에 설정이 완료되었음을 표시

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //반복을 위한 초기화 작업

									//설정 완료 후 메뉴로 돌아가기 때문에 메뉴 미리 출력
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break; //while문 빠져나가기
								}
							}
						}
					}
				}
			}
		}
	}
	return 0;
}

void FanOn(void) //워터펌프 작동 
{
	digitalWrite(A_1A, HIGH);
	digitalWrite(A_2A, LOW);
}

void FanOff(void) //워터펌프 중지
{
	digitalWrite(A_1A, LOW);
	digitalWrite(A_2A, LOW);
}