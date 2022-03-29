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

//keypad (11��)
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
#define PW_S 14 //��ũ�� �� ���� ��ư

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

#define MAX_KEY_BT_NUM 10 //Ű�е� 0~9���� �� 10���� ���Ѵ�.

#define A_1A  15 //water pump  
#define A_2A  18 //water pump

#define MT_P 7 //dc motor
#define MT_N 19 //dc motor

//dc motor control (����)
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

const int KeypadTable[MAX_KEY_BT_NUM] = { PW0, PW1,PW2,PW3,PW4,PW5,PW6,PW7,PW8,PW9 }; //Ű�е� �迭�� 0~9���� ����

int set_tem, set_shum, set_water = 0; //�µ�, ������, ���� ������ ����
int tem, hum = 0; //�Ĺ� �ֺ� �µ��� ���� �� ����
int plant_state = 0; //�Ĺ��� ���� ���� ����
int water_result, shum_result = 0; //�������� �����̰� ��� ������ ������ �������� %�� �ٲ� �� ���� 
int mcount = 2; //���� ������ ���� ������ �� �ʿ��� ����

int tem2[2], hum2[2], water3[3]; //����ڷκ��� �Է¹��� �µ�, ������, ���� ����

void FanOn(void); //water pump �۵� �Լ�
void FanOff(void); //water pump ���� �Լ�


//DC���� ���� �Լ���
void MotorStop() //���͸� ���ߴ� �Լ�
{
	softPwmWrite(MT_P, 0);
	softPwmWrite(MT_N, 0);
}
void MotorControl(unsigned char speed, unsigned char rotate) //���͸� ������ �Լ�
{
	if (rotate == LEFT_ROTATE) { //���� �������� ȸ��
		digitalWrite(MT_P, LOW);
		softPwmWrite(MT_N, speed);
	}
	else if (rotate == RIGHT_ROTATE) { //���� ���������� ȸ��
		digitalWrite(MT_N, LOW);
		softPwmWrite(MT_P, speed);
	}
}

//Buzzer ���� �Լ���
unsigned int SevenScale(unsigned char scale) //���ϴ� ���� ���� ����
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

void Change_FREQ(unsigned int freq) //�Ҹ��� �ٲٴ� �Լ�
{
	softToneWrite(BUZZER_PIN, freq);
}

void STOP_FREQ(void) //�Ҹ��� ���ߴ� �Լ�
{
	softToneWrite(BUZZER_PIN, 0);
}

void Buzzer_Init(void) //Buzzer ����
{
	softToneCreate(BUZZER_PIN);
	STOP_FREQ();
}


//ADC �Լ� --- ���� 3���� ����
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

//�½����� �޾ƿ��� �Լ�
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
	int nCdsChannel = 0; //���������� ä���� 0���� ����
	int soilChannel = 1; //������������ ä���� 1�� ����
	int moistChannel = 2; //���������� ä���� 2�� ����

	int nCdsValue = 0; //���� �ʱⰪ
	int soilValue = 0; //������ �ʱⰪ
	int moistValue = 0; //�������� �ʱⰪ

	int disp1; //LCD ����
	int S_count = 0, S_count4 = 0; //�޴� 4���� ���� count����, �޴� 3���� ���� count����
	//�޴�1�� ������ ���°� ����, �޴�2�� ������ ���°� ����, �޴�3�� ������ ���°� ����, �޴�4�� ������ ���°� ����
	int count1 = 0, count2 = 0, count3 = 0, count4 = 0; 
	int count = 0; //�µ�, ������, ���� �ڸ��� ���� ����
	int setting = 0; //���� �޴� ���°� ����


	if (wiringPiSetupGpio() == -1) {
		fprintf(stdout, "Not start wiringPi: %s\n", strerror(errno));
		return 1;
	}

	if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
		fprintf(stdout, "wiringPiSPISetup Failed: %s\n", strerror(errno));
		return 1;
	}

	//DC���� OUTPUT ���� �� ����
	pinMode(MT_P, OUTPUT);
	pinMode(MT_N, OUTPUT);
	softPwmCreate(MT_P, 0, 100);
	softPwmCreate(MT_N, 0, 100);

	//�������� OUTPUT ����
	pinMode(A_1A, OUTPUT);
	pinMode(A_2A, OUTPUT);

	//ADC OUTPUT ����
	pinMode(CS_MCP3208, OUTPUT);

	//Ű�е� INPUT ����
	for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
		pinMode(KeypadTable[i], INPUT);
	}

	disp1 = lcdInit(2, 16, 4, 23, 24, 20, 21, 12, 16, 0, 0, 0, 0); //LCD �ʱ�ȭ
	lcdClear(disp1); //LCD ȭ�� �����

	Buzzer_Init(); //���� �ʱ�ȭ
	read_dht11_dat(); //�½����� �޾ƿ���

	lcdClear(disp1); //LCD ȭ�� �����
	lcdPosition(disp1, 0, 0);
	lcdPrintf(disp1, "Tem:%d", tem); //ù ��°�ٿ� ������ �޾ƿ� �µ��� ��� 
	lcdPosition(disp1, 7, 0);
	lcdPrintf(disp1, "Hum:%d", hum); //ù ��°�ٿ� ������ �޾ƿ� ������ ���
	lcdPosition(disp1, 0, 1);
	lcdPrintf(disp1, "PlantState:%d", plant_state); //�� ��°�ٿ� ���õ� �Ĺ� ���°� �������� ���
	lcdPosition(disp1, 15, 1);
	lcdPrintf(disp1, "S"); //�� ��°�ٿ� ������ ��Ÿ���� S ���

	while (1) {
		MotorStop(); //DC���ʹ� �׻� ���� ���¿��� ����
		FanOff(); //���������� �׻� ���� ���¿��� ����

		nCdsValue = ReadMcp3208ADC(nCdsChannel); //�������� �������� �޾ƿͼ� ADC�� ��ȯ �� �� ���� 
		soilValue = ReadMcp3208ADC(soilChannel); //�������� ���������� �޾ƿͼ� ADC�� ��ȯ �� �� ���� 
		moistValue = ReadMcp3208ADC(moistChannel); //�������� �������� �޾ƿͼ� ADC�� ��ȯ �� �� ���� 

		soilValue = (soilValue / 100)*2.5; //������ �޾ƿ� ���������� %�� ��Ÿ���� ���� ���� 

		printf("Soil Sensor Value = % u\n", soilValue); //������ �� ȭ������ ���
		printf("Moist Sensor Value = % u\n", moistValue); //������ ȭ������ ���
		printf("Cds : %u \n", nCdsValue); //������ ȭ������ ���

		read_dht11_dat(); //�½����� ����ؼ� ������Ʈ�ϱ� ���� �޾ƿ���

		//LCD �ʱ� ȭ��
		lcdPosition(disp1, 0, 0);
		lcdPrintf(disp1, "Tem:%d", tem); //ù ��°�ٿ� �µ� ���
		lcdPosition(disp1, 7, 0);
		lcdPrintf(disp1, "Hum:%d", hum); //ù ��°�ٿ� ���� ���
		lcdPosition(disp1, 0, 1);
		lcdPrintf(disp1, "PlantState:%d", plant_state); //�� ��°�ٿ� ������ �Ĺ����� ���
		lcdPosition(disp1, 15, 1);
		lcdPrintf(disp1, "S"); //�� ��°�ٿ� ������ ��Ÿ���� S ���

		//�����ε带 ��Ʈ�� �ϴ� IF��
		if (set_tem != 0) { //�����ε� ��Ʈ���� ���� �µ����� �����Ǿ� ���� ��
			/*�����ε� ������
			�������� 3000�̻��̰� ���� �µ��� ����ڰ� ������ �µ����� ���� ��, mcount�� 2�� ��
			mcount�� 1���� 2������ ���������ν� ������ �����ε�� ���̻� ������ ������ �� �� ���� �ø��� ���۸� ����������.*/
			if (nCdsValue >= 3000 && tem >= set_tem && mcount == 2) { 
				MotorStop(); //���� ���߱�
				delay(2000); //���� ���¸� 2������ ����
				MotorControl(50, LEFT_ROTATE); //���͸� �������� ȸ�������ν� �����ε尡 ��������.
				delay(9000);//9�� ���� �����ε带 ������.
				MotorStop(); //9�� �� �����ε带 �����.
				mcount = 1; //���� ������ �����ε� �ø��� ������ �������� mcount�� 1�� ����
			}
			/*�����ε� �ø���
			�������� 3000�̸��̰� ���� �µ��� ����ڰ� ������ �µ����� ���� ��, mcount�� 1�� ��
			mcount�� 1���� 2������ ���������ν� �÷��� �����ε�� ���̻� �ø��� ������ �� �� ���� ������ ���۸� ����������.*/
			if (nCdsValue < 3000 && tem < set_tem && mcount == 1) { 
				MotorStop(); //���� ���߱�
				delay(2000); //���� ���¸� 2������ ����
				MotorControl(50, RIGHT_ROTATE); //���͸� �������� ȸ�������ν� �����ε尡 �ö󰣴�.
				delay(9000); //9�� ���� �����ε带 �������� ������ �ø� ���� 9�ʸ� �ش�.
				MotorStop(); //9�� �� �����ε带 �����.
				mcount = 2; //���� ������ �����ε� ������ ������ �������� mcount�� 2�� ����
			}
		}

		//�������� ���� �� ���������� �����ϴ� �κ�
		if (set_shum != 0) { //���������� ���۽�Ű�� ���� ���������� �����Ǿ����� ��
			if (soilValue < set_shum) { //���������� ����ڰ� ������ ������ ������ ���� ���
				lcdClear(disp1); //LCD �����
				lcdPosition(disp1, 0, 0);
				delay(2000);
				lcdPrintf(disp1, "Watering %dml", set_water); //LCD�� ����ڰ� �����س��� ���簪��ŭ ���� �شٴ� ȭ���� ���´�.
				lcdPosition(disp1, 0, 1);
				lcdPuts(disp1, "Please Wait..."); 

				FanOn(); //���������� ���۽�Ų��.
				delay(water_result); //����ڰ� ������ ���簪�� ����ؼ� �������� �����̰� ��ŭ �����̸� �־ ������ �����Ѵ�.
				lcdClear(disp1);
			}
		}	

		//�������� ������ ������ �︰��. 
		if (moistValue < 1500){ //�������� 1500�̸��� ���
			lcdClear(disp1); 
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Water shortage"); //LCD�� �������� �˷��ִ� �˸��� ���� ���� ������� �˷��ش�.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Put Water...");

			//�� ���� ������� �︰��.
			Change_FREQ(SevenScale(5));
			delay(300);
			STOP_FREQ();
			Change_FREQ(SevenScale(1));
			delay(300);
			STOP_FREQ();
			delay(1000);
		}

		//�ڵ����� ���� �ְ����� �� ���� �������� ���� �ְ����� �� 
		//�������� ���ֱ� 1��(���� �� ��)
		if (!digitalRead(PW1)) { //1�� Ű�е带 ������ ��
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 50ml"); //LCD�� 50ml�� ���� �شٴ� ȭ���� ����.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD�� ��ٸ���� ����..

			FanOn(); //���� ������ ���۽�Ų��.
			delay(5000); //�������� 1�ʿ� 10ml�� ���� ������ ������ 50ml�� ���� 5���� �����̸� �ش�.
			lcdClear(disp1);
		}
		//�������� ���ֱ� 2��(�߰� �� ��)
		if (!digitalRead(PW2)) { //2�� Ű�е带 ������ ��
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 100ml"); //LCD�� 100ml�� ���� �شٴ� ȭ���� ����.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD�� ��ٸ���� ����..

			FanOn(); //���� ������ ���۽�Ų��.
			delay(10000); //�������� 1�ʿ� 10ml�� ���� ������ ������ 100ml�� ���� 10���� �����̸� �ش�.
			lcdClear(disp1);
		}
		//�������� ���ֱ� 3��(���� �� ��)
		if (!digitalRead(PW3)) { //3�� Ű�е带 ������ ��
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "Watering 150ml"); //LCD�� 150ml�� ���� �شٴ� ȭ���� ����.
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "Please Wait..."); //LCD�� ��ٸ���� ����..

			FanOn(); //���� ������ ���۽�Ų��.
			delay(15000); //�������� 1�ʿ� 10ml�� ���� ������ ������ 150ml�� ���� 15���� �����̸� �ش�.
			lcdClear(disp1);
		}

		//�Ĺ��� ���� ���� ������ �� �ڼ��ϰ� �˰����� ��
		if (!digitalRead(PW0)) { //Ű�е� 0���� ������ ��
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();
			setting = 2; //setting�� 2�� �Ѵ�.
			lcdClear(disp1);

			while (setting == 2) { //setting�� 2�� ��쿡 ��� �ݺ�
				//lcdClear(disp1);
				lcdPosition(disp1, 0, 0);
				lcdPrintf(disp1, "Tem:%d", set_tem); //������ �µ� �� ���
				lcdPosition(disp1, 7, 0);
				lcdPrintf(disp1, "SHum:%d", set_shum); //������ ������ �� ���
				lcdPosition(disp1, 0, 1);
				lcdPrintf(disp1, "Water:%d", set_water); //������ ���� �� ���
				lcdPosition(disp1, 15, 1);
				lcdPuts(disp1, "B"); //back�� �� �ֵ��� B�� ǥ��

				if (!digitalRead(PW_S)) { //���� ���� B�� �ش��ϴ� ��ư(PW_S)�� ������ �� �ʱ�ȭ������ ���ư���
					setting = 0; //while���� ���������� ���� setting�� 0
					delay(500); //�ʱ�ȭ���� setting�� �ǹ��ϴ� S�� �������ʰ� �ϱ� ���� delay 0.5�ʷ� ��ư������ �����ش�.
					lcdClear(disp1);
					break; //while�� ���������� (�ʱ�ȭ������ ���ư� �� �ִ�.)
				}
			}
		}

		//���� ������ ���� �˰����� ��
		if (!digitalRead(PW4)) { //Ű�е� 4���� ������ ��
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			setting = 4; //setting�� 4�� �Ѵ�.
			lcdClear(disp1);

			while (setting == 4) { //setting�� 4�� ��쿡 ��� �ݺ�
				//lcdClear(disp1);
				lcdPosition(disp1, 0, 0);
				lcdPrintf(disp1, "Current Value");
				lcdPrintf(disp1, "SoilHum:%d", soilValue); //���� ������ �� ���
				lcdPosition(disp1, 0, 10);
				lcdPrintf(disp1, "%");
				lcdPosition(disp1, 15, 1);
				lcdPuts(disp1, "B"); //back�� �� �ֵ��� B�� ǥ��

				if (!digitalRead(PW_S)) { //���� ���� B�� �ش��ϴ� ��ư(PW_S)�� ������ �� �ʱ�ȭ������ ���ư���
					setting = 0; //while���� ���������� ���� setting�� 0
					delay(500); //�ʱ�ȭ���� setting�� �ǹ��ϴ� S�� �������ʰ� �ϱ� ���� delay 0.5�ʷ� ��ư������ �����ش�.
					lcdClear(disp1);
					break; //while�� ���������� (�ʱ�ȭ������ ���ư� �� �ִ�.)
				}
			}
		}

		//SET �޴��� ����
		if (!digitalRead(PW_S)) { //PW_S ��ư�� ���� �� set �޴��� ����.
			Change_FREQ(SevenScale(0));
			delay(200);
			STOP_FREQ();

			setting = 1; //setting���� 1�� �ش�.

			lcdClear(disp1);
			lcdPosition(disp1, 0, 0);
			lcdPuts(disp1, "1.Plant1");
			lcdPosition(disp1, 0, 1);
			lcdPuts(disp1, "2.Plant2");
			lcdPosition(disp1, 15, 1);
			lcdPuts(disp1, "D");

			delay(500); //set �޴��� �� ���� S��ư ������ ��ũ�� S��ư ������ ��ġ�� �ʰ� delay

			while (setting == 1) { //setting���� 1�� �� �ݺ�
				if (!digitalRead(PW_S)) { //S��ư�� ������ �� ��ũ��
					Change_FREQ(SevenScale(9));
					delay(200);
					STOP_FREQ();

					S_count++; //��ư�� ���� ������ S_count�� 1�� �ø���.
					S_count %= 4; //�޴��� 4���̱� ������ 4�� ���� �������� ���Ѵ�.

					switch (S_count) { //S_count�� ����
					case 0: //S_count���� 0�� �� �޴�1�� 2�� ���´�.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "1.Plant1");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "2.Plant2");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 1: //S_count���� 1�� �� �޴�2�� 3�� ���´�.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "2.Plant2");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "3.Plant3");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 2: //S_count���� 2�� �� �޴�3�� 4�� ���´�.
						lcdClear(disp1);
						lcdPosition(disp1, 0, 0);
						lcdPuts(disp1, "3.Plant3");
						lcdPosition(disp1, 0, 1);
						lcdPuts(disp1, "4.User Set");
						lcdPosition(disp1, 15, 1);
						lcdPuts(disp1, "D");
						break;
					case 3: //S_count���� 3�� �� �޴�4�� 1�� ���´�.
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

				//�ڷΰ��⸦ �ϰ����� ��
				if (!digitalRead(PW0)) { //PW0�� ������ BACK
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					S_count = 0; //�޴��� 1���� ���� ���� S_count=0���� �ʱ�ȭ

					//����ȭ������ ���ư��� ������ ����ȭ�� LCD�� ���
					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPrintf(disp1, "Tem:%d", tem); //�µ��� ���
					lcdPosition(disp1, 7, 0);
					lcdPrintf(disp1, "Hum:%d", hum); //�������� ���
					lcdPosition(disp1, 0, 1);
					lcdPrintf(disp1, "PlantState:%d", plant_state); //�Ĺ����°� ���
					lcdPosition(disp1, 15, 1);
					lcdPrintf(disp1, "S");

					delay(500); //���ο��� PW0 �۾��� ��ġ�� �ʰ� �ϱ� ���� delay
					break;
				}

				//PW1�� ������ �Ĺ�����1 �� �����Ѵ�.
				if (!digitalRead(PW1)) { //1�� Ű�е尡 ������ ��
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count1 = 1; //count1�� 1�� �ش�.

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:10"); //LCD�� �Ĺ�����1�� �µ��� ���� �����ֱ�
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:10"); //LCD�� �Ĺ�����1�� �������� ���� �����ֱ�
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:50"); //LCD�� �Ĺ�����1�� ���簪 ���� �����ֱ�
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD�� �Ĺ� ���� ������ ���� S �����ֱ�

					while (count1 == 1) { //count1�� �� �ݺ�
						if (!digitalRead(PW_S)) { //PW_S�� ������ �Ĺ�����1 ���� �Ϸ�
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}
							plant_state = 1; //�Ĺ� ���¸� 1�� �ٲ۴�.

							//�ÿ��� ���� ���� �ش������� �־���.
							set_tem = 10; //�µ��� 10���� �����Ѵ�.
							set_shum = 10; //�������� 10%�� �����Ѵ�.
							set_water = 50; //�� ���� 50ml�� �����Ѵ�.

							//�� ���� 1�ʿ� 10ml�̱� ������ 50ml�� 5���̹Ƿ� ���� ���簪���� 100�� ���ϰ� ���� ����ø��� �ð��� �����ؼ� 2��(2000)�� �߰��Ѵ�.
							water_result = set_water * 100 + 2000; //�� �� �����̰� ����

							//��� Ȯ�� �κ�
							printf("%d\n", set_tem);
							printf("%d\n", set_shum);
							printf("%d\n", set_water);

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD�� ������ �� �Ǿ����� �����ؤ�.

							S_count = 0; // �޴��κ��� 1���� �������� �ϱ� ���� S_count�� 0���� �ʱ�ȭ
							delay(2000);
							lcdClear(disp1);
							setting = 0; //setting�� 0���� �ʱ�ȭ
							break; //���� �� while���� ����������.

						}

						//�ڷΰ���
						if (!digitalRead(PW0)) { //PW0�� ������ �ڷΰ��⸦ �Ѵ�.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //�޴��� 1���� �������� S_count�� 0���� �ʱ�ȭ

							//�޴� �κ� �̸� LCD�� ���
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1"); 
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while�� ���� ������
						}
					}
				}

				//PW2�� ������ �Ĺ�����2 �� �����Ѵ�.
				if (!digitalRead(PW2)) { //2�� Ű�е尡 ������ ��
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count2 = 1; //count2�� 1�� �ش�

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:20"); //LCD�� �Ĺ�����2�� �µ��� ���� �����ֱ�
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:50"); //LCD�� �Ĺ�����2�� �������� ���� �����ֱ�
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:100"); //LCD�� �Ĺ�����2�� ���簪 ���� �����ֱ�
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD�� �Ĺ� ���� ������ ���� S �����ֱ�

					while (count2 == 1) { //count2�� 1�� �� �ݺ�
						if (!digitalRead(PW_S)) { //PW_S�� ������ �Ĺ�����2 ���� �Ϸ�
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}

							plant_state = 2; //�Ĺ� ���¸� 2�� �ٲ۴�.

							//�ÿ��� ���� ���� �ش������� �־���.
							set_tem = 20; //�µ��� 20���� �����Ѵ�.
							set_shum = 50; //�������� 50%�� �����Ѵ�.
							set_water = 100; //�� ���� 100ml�� �����Ѵ�.

							//�� ���� 1�ʿ� 10ml�̱� ������ 50ml�� 5���̹Ƿ� ���� ���簪���� 100�� ���ϰ� ���� ����ø��� �ð��� �����ؼ� 2��(2000)�� �߰��Ѵ�.
							water_result = set_water * 100 + 2000; //�� �� �����̰� ����

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD�� ������ �� �Ǿ����� �����ش�.

							S_count = 0; // �޴��κ��� 1���� �������� �ϱ� ���� S_count�� 0���� �ʱ�ȭ
							delay(2000);
							lcdClear(disp1); 
							setting = 0; //setting�� 0���� �ʱ�ȭ
							break; //���� �� while���� ����������.
						}

						//�ڷΰ���
						if (!digitalRead(PW0)) { //PW0�� ������ �ڷΰ��⸦ �Ѵ�.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //�޴��� 1���� �������� S_count�� 0���� �ʱ�ȭ

							//�޴� �κ� �̸� LCD�� ���
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while�� ���� ������
						}
					}
				}

				//PW3�� ������ �Ĺ�����3 �� �����Ѵ�.
				if (!digitalRead(PW3)) {
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count3 = 1; //count3�� 1�� �ش�.

					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "Tem:40"); //LCD�� �Ĺ�����3�� �µ��� ���� �����ֱ�
					lcdPosition(disp1, 7, 0);
					lcdPuts(disp1, "SHum:80"); //LCD�� �Ĺ�����3�� �������� ���� �����ֱ�
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "Water:150"); //LCD�� �Ĺ�����3�� ���簪 ���� �����ֱ�
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "S"); //LCD�� �Ĺ� ���� ������ ���� S �����ֱ�

					while (count3 == 1) { //count3�� 1�� �� �ݺ�
						if (!digitalRead(PW_S)) { //PW_S�� ������ �Ĺ�����3 ���� �Ϸ�
							for (int i = 0; i < 3; i++) {
								Change_FREQ(SevenScale(i));
								delay(200);
								STOP_FREQ();
							}
							plant_state = 3; //�Ĺ� ���¸� 3���� �ٲ۴�.

							//�ÿ��� ���� ���� �ش������� �־���.
							set_tem = 40; //�µ��� 40���� �����Ѵ�.
							set_shum = 80; //�������� 80%�� �����Ѵ�.
							set_water = 150; //�� ���� 150ml�� �����Ѵ�.

							//�� ���� 1�ʿ� 10ml�̱� ������ 50ml�� 5���̹Ƿ� ���� ���簪���� 100�� ���ϰ� ���� ����ø��� �ð��� �����ؼ� 2��(2000)�� �߰��Ѵ�.
							water_result = set_water * 100 + 2000; //�� �� �����̰� ����

							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "Set Complete"); //LCD�� ������ �� �Ǿ����� �����ش�.

							S_count = 0; // �޴��κ��� 1���� �������� �ϱ� ���� S_count�� 0���� �ʱ�ȭ
							delay(2000);
							lcdClear(disp1);
							setting = 0; //setting�� 0���� �ʱ�ȭ
							break; //���� �� while���� ����������.
						}

						//�ڷΰ���
						if (!digitalRead(PW0)) { //PW0�� ������ �ڷΰ��⸦ �Ѵ�.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //�޴��� 1���� �������� S_count�� 0���� �ʱ�ȭ

							//�޴� �κ� �̸� LCD�� ���
							lcdClear(disp1);
							lcdPosition(disp1, 0, 0);
							lcdPuts(disp1, "1.Plant1");
							lcdPosition(disp1, 0, 1);
							lcdPuts(disp1, "2.Plant2");
							lcdPosition(disp1, 15, 1);
							lcdPuts(disp1, "D");
							break; //while�� ���� ������
						}
					}
				}

				//PW4�� ������ ����ڼ����� �� �� �ִ�.
				if ((!digitalRead(PW4))) { //4�� Ű�е尡 ������ ��
					Change_FREQ(SevenScale(0));
					delay(200);
					STOP_FREQ();

					count4 = 1; //count4�� 1�� �ش�.

					//����� ������ �޴��� LCD�� �����ش�.
					lcdClear(disp1);
					lcdPosition(disp1, 0, 0);
					lcdPuts(disp1, "1.Tem Set"); //�µ� ���� ���� 
					lcdPosition(disp1, 0, 1);
					lcdPuts(disp1, "2.Hum Set"); //��� ���� ���� ����
					lcdPosition(disp1, 15, 1);
					lcdPuts(disp1, "D"); //��ũ�� D

					while (count4 == 1) {
						if (!digitalRead(PW_S)) { //scroll
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count4++; //PW_S�� ���� ������ S_count4�� ����
							S_count4 %= 3; //�޴��� 3���̱� ������ 3���� ���� �������� ���

							switch (S_count4) { //S_count4 ����
							case 0: //S_count4�� 0�� �� �޴�1,2 �����ֱ�
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "1.Tem Set"); //1�� �޴�: �µ� ����
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "2.Hum Set"); //1�� �޴�: ������ ����
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //��ũ��
								break;
							case 1: //S_count4�� 1�� ��
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "2.Hum Set"); //2�� �޴�: ������ ����
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "3.Water Set"); //3�� �޴�: ���� ����
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //��ũ��
								break;
							case 2: //S_count4�� 2�� ��
								lcdClear(disp1);
								lcdPosition(disp1, 0, 0);
								lcdPuts(disp1, "3.Water Set"); //3�� �޴�: ���� ����
								lcdPosition(disp1, 0, 1);
								lcdPuts(disp1, "1.Tem Set"); //1�� �޴�: �µ� ����
								lcdPosition(disp1, 15, 1);
								lcdPuts(disp1, "D"); //��ũ��
								break;
							}
						}

						if (!digitalRead(PW0)) { //PW0�� ������ �ڷΰ��⸦ �Ѵ�.
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							S_count = 0; //�޴��� 1���� �������� S_count�� 0���� �ʱ�ȭ
							break; //while�� ���� ������
						}


						//PW1�� ������ ����ڰ� ���� �µ��� �����Ѵ�.
						if (!digitalRead(PW1)) { //PW1�� ������ �µ� ����
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//�µ� �Է��� ���� LCD �Է�ȭ�� ���
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

							while (count < 2) { //count�� 2�̸��� �� �ݺ�, count�� �µ� �ڸ���
								for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //��ư ä�͸� ������ ���� ���� ��ư�� ������ �ʾ��� ���� �Է��� ���� �Ͽ���.
											if (((!digitalRead(PW_S)) == LOW)){ //S�� ���� �Է��� �ȵǰ� �Ͽ���.
												tem2[count] = i; //����� ���� �µ� ������ ��ư �� ����
												printf("�Է���: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->�� 'C ���̿� �µ��� �ԷµǴ� ���� Ȯ���� �� �ִ�.
												lcdPrintf(disp1, "%d", i); //����ڰ� �Է��� ��ư���� ���(���ڸ���)
												count++; //count�� ++�ؼ� �ڸ��� ����
												continue; //�ݺ�
											}
										}
									}
									else {
										continue; //�ݺ�
									}
								}
							}

							while (count == 2) { //count�� 2�� ��, count�� �µ� �ڸ��� --> �µ��� ���ڸ��� �ǹ̰� ���ٴ� ������ ��� ���ڸ��� ������ ����
								if ((!digitalRead(PW_S)) == HIGH && count == 2) { //S�� ������ �µ��� ���ڸ� ���̸� �µ� ����
									for (int i = 0; i < 2; i++) {
										printf("%d", tem2[i]);
									}
									plant_state = 4; //�Ĺ� ���¸� ����� ������ 4�� �ٲ۴�.

									//�µ��� �迭�� �޾ұ� ������ �ε��� 0�� ���� �ڸ����� 10�� ���ϰ� �ε���1�� ���� �ڸ����� 1�� ���ؼ� �̵��� ���ߴ�.
									set_tem = tem2[0] * 10 + tem2[1] * 1; 

									printf("\nTEMP�� �����Ǿ����ϴ�.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD�� ������ �Ϸ�Ǿ����� ǥ��

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //�ݺ��� ���� �ʱ�ȭ �۾�

									//���� �Ϸ� �� �޴��� ���ư��� ������ �޴� �̸� ���
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break; //while�� ����������
								}
							}
						}

						//PW2�� ������ ����ڰ� ���� ��� ������ �����Ѵ�.
						if (!digitalRead(PW2)) { //PW2�� ������ ������ ����
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//������ �Է��� ���� LCD �Է�ȭ�� ���
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

							while (count < 2) { //count�� 2�̸��� �� �ݺ�, count�� ������ �ڸ���
								for (int i = 0; i < MAX_KEY_BT_NUM; i++) {
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //��ư ä�͸� ������ ���� ���� ��ư�� ������ �ʾ��� ���� �Է��� ���� �Ͽ���.
											if (((!digitalRead(PW_S)) == LOW)) { //S�� ���� �Է��� �ȵǰ� �Ͽ���.
												hum2[count] = i; //����� ���� ������ ������ ��ư �� ����
												printf("�Է���: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->�� % ���̿� �������� �ԷµǴ� ���� Ȯ���� �� �ִ�.
												lcdPrintf(disp1, "%d", i); //����ڰ� �Է��� ��ư���� ���(���ڸ���)
												count++; //count�� ++�ؼ� �ڸ��� ����
												continue; //�ݺ�
											}
										}
									}
									else {
										continue; //�ݺ�
									}
								}
							}

							while (count == 2) { //count�� 2�� ��, count�� ������ �ڸ��� 
								if ((!digitalRead(PW_S)) == HIGH && count == 2) { //S�� ������ �������� ���ڸ� ���̸� ������ ����
									for (int i = 0; i < 2; i++) {
										printf("%d", hum2[i]);
									}
									plant_state = 4; //�Ĺ� ���¸� ����� ������ 4�� �ٲ۴�.

									//�������� �迭�� �޾ұ� ������ �ε��� 0�� ���� �ڸ����� 10�� ���ϰ� �ε���1�� ���� �ڸ����� 1�� ���ؼ� �̵��� ���ߴ�.
									set_shum = hum2[0] * 10 + hum2[1] * 1;

									printf("\nHum�� �����Ǿ����ϴ�.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD�� ������ �Ϸ�Ǿ����� ǥ��

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //�ݺ��� ���� �ʱ�ȭ �۾�

									//���� �Ϸ� �� �޴��� ���ư��� ������ �޴� �̸� ���
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break;  //while�� ����������
								}
							}
						}

						//PW3�� ������ ����ڰ� ���� ������ �����Ѵ�.
						if (!digitalRead(PW3)) { //PW3�� ������ ���� ����
							Change_FREQ(SevenScale(0));
							delay(200);
							STOP_FREQ();

							//�µ� �Է��� ���� LCD �Է�ȭ�� ���
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

							while (count < 3) { //count�� 3�̸��� �� �ݺ�, count�� ���� �ڸ��� --> �� �ڸ����� �Է� ����
								if (!digitalRead(PW_S) == HIGH) { // //S�� ������ �� ���ڸ����̰ų� ���ڸ����̸� �ݺ��� ������.
									if (count == 2 || count == 3) { 
										break;
									}
								}

								for (int i = 0; i < MAX_KEY_BT_NUM; i++) { //Ű�е� �Է�
									if ((!digitalRead(KeypadTable[i]))) {
										if ((!digitalRead(KeypadTable[i])) == HIGH) {
											printf("\n");
										}
										if ((!digitalRead(KeypadTable[i])) == LOW) { //��ư ä�͸� ������ ���� ���� ��ư�� ������ �ʾ��� ���� �Է��� ���� �Ͽ���.
											if (((!digitalRead(PW_S)) == LOW)) { //S�� ���� �Է��� �ȵǰ� �Ͽ���.
												water3[count] = i; //����� ���� ���� ������ ��ư �� ����
												printf("�Է���: %d\n", i);
												Change_FREQ(SevenScale(i));
												delay(200);
												STOP_FREQ();

												lcdPosition(disp1, count + 3, 1); //LCD ->�� ml ���̿� ������ �ԷµǴ� ���� Ȯ���� �� �ִ�.
												lcdPrintf(disp1, "%d", i); //����ڰ� �Է��� ��ư���� ���(���ڸ���)
												count++; //count�� ++�ؼ� �ڸ��� ����
												continue;  //�ݺ�
											}
										}
									}
									else {
										continue;  //�ݺ�
									}
								}
							}

							//count�� 2�̰ų� 3�� ��, count�� ���� �ڸ��� --> ������ 3�ڸ������� �� �� �־�� ���� �� ���Ƽ� �̷��� �����ߴ�.
							while (count == 2 || count == 3) { 
								if ((!digitalRead(PW_S)) == HIGH && (count == 2 || count == 3)) { //S�� ������ �µ��� ���ڸ� ���ų� ���ڸ� ���̸� ���� ����
									for (int i = 0; i < 3; i++) {
										printf("%d", water3[i]);
									}
									plant_state = 4; //�Ĺ� ���¸� ����� ������ 4�� �ٲ۴�.

									//�ڸ����� ���̳Ŀ� ���� ������ �޶����� ������ count�� 2�϶��� 3�϶��� ��������.
									if (count == 2) { //���ڸ����϶�
										set_water = water3[0] * 10 + water3[1] * 1; //�ε���0�� ���� �ڸ����̹Ƿ� 10�� ���ϰ� �ε���1�� ���� �ڸ����̹Ƿ� 1�� ���Ѵ�.
									}
									if (count == 3) { //���ڸ����϶�
										//�ε���0�� ���� �ڸ����̹Ƿ� 100�� ���ϰ� �ε���1�� ���� �ڸ����̹Ƿ� 10�� ���Ѵ�. �ε���2�� ���� �ڸ����̹Ƿ� 1�� ���Ѵ�.
										set_water = water3[0] * 100 + water3[1] * 10 + water3[2] * 1;
									}

									water_result = set_water * 100 + 2000; //������ �����ϴ� ����� �������̱� ������ ���� �����̸� ����Ѵ�.

									printf("\nWater�� �����Ǿ����ϴ�.\n");

									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "Set Complete"); //LCD�� ������ �Ϸ�Ǿ����� ǥ��

									for (int i = 0; i < 3; i++) {
										Change_FREQ(SevenScale(i));
										delay(200);
										STOP_FREQ();
									}
									delay(2000);
									count = 0; //�ݺ��� ���� �ʱ�ȭ �۾�

									//���� �Ϸ� �� �޴��� ���ư��� ������ �޴� �̸� ���
									lcdClear(disp1);
									lcdPosition(disp1, 0, 0);
									lcdPuts(disp1, "1.Tem Set");
									lcdPosition(disp1, 0, 1);
									lcdPuts(disp1, "2.Hum Set");
									lcdPosition(disp1, 15, 1);
									lcdPuts(disp1, "D");
									break; //while�� ����������
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

void FanOn(void) //�������� �۵� 
{
	digitalWrite(A_1A, HIGH);
	digitalWrite(A_2A, LOW);
}

void FanOff(void) //�������� ����
{
	digitalWrite(A_1A, LOW);
	digitalWrite(A_2A, LOW);
}