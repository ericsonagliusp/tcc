#ínicio do Código Python do Controlador de Temperatura

#Bibliotecas usadas:
from opcua import Client
import time
from random import randint
import paho.mqtt.client as mqtt
import MAX6675.MAX6675 as MAX
import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn 


#Definição dos parâmetros do sensor de temperatura termopar K - Pinos da GPIO (GPIO.BCM) usados:
CS = 24
CSK = 25
DO = 18

saidarele=20 #Pino da saída (GPIO.BCM) relé.
saidapwm=17 #Pino da saída (GPIO.BCM) pwm.
saida420=21 #Pino da saída (GPIO.BCM) 420.
f=10 #Ffrequência PWM.

sensortemp = MAX.MAX6675(CSK, CS, DO)

#Para inicializar Node-Red no boot: sudo systemctl enable nodered.service.
#Para NãO inicializar mais o Node-Red no boot: sudo systemctl disable nodered.service.
#Para inicializar Node-Red: sudo systemctl start nodered.
#Para encerrar Node-Red: sudo systemctl stop nodered.

time.sleep(1) #Para aguardar a inicialização do Node-Red

counter=0 #Contador para que se tente novamente conectar o OPC-UA e o MQTT, caso exista algum erro, e encerre o código caso o erro persista para que o Node-Red possa ser reinicializado.
zerocounter=0 #Zera a variável: "counter" caso o loop volte a rodar.

#Definição de mais algumas variáveis:
PT100X=0 #Taxa de inclinação da temperatura lida pelo sensor PT100 em função da tensão de 0 a 5 V.
PT100Y=0 #Temperatura lida pelo sensor PT100 quando a tensão de 0 a 5 V.


print("Ínicio do Código Controlador de Temperatura")

while (counter<3):
      
      try:
        sensortemp = MAX.MAX6675(CSK, CS, DO)
        #Conexão com o servidor OPCUA:     
        url = "opc.tcp://127.0.0.1:53880"       
         
        broker_address = "127.0.0.1" #Endereço IP para comunicação MQTT.        
        client = Client(url)
        client.connect()
        print("Client Connected OPC-UA")
        
        
        
        #Conexão com o broker do mqtt Mosquitto:
        connected = False

        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                connected = True
                print("Connected MQTT")
            else:
                print("Not Able To Connect")

        

        cliente = mqtt.Client("P1") 
        cliente.on_connect = on_connect
        cliente.connect(broker_address, port=1883)
        
        

        while True:
              
            #Função para a medida do conversor AD: ADS1115:
            def Read_AD(channel): 

               
                #Create the I2C bus
                i2c = busio.I2C(board.SCL, board.SDA)

                # Create the ADC object using the I2C bus
                ads = ADS.ADS1115(i2c)

                # Create single-ended input on channel 0
                chan = AnalogIn(ads, channel)

                # Create differential input between channel 0 and 1
                #chan = AnalogIn(ads, ADS.P0, ADS.P1)
               
                #Para continuar o processo, mesmo com o: O/S Error: Errno 121
                success = False
                caught_exception = None
                for _ in range(10):
                    try:
                        print('try')
                        analog=chan.voltage #Leitua do valor analógico.            
                        success = True
                        break
                    except IOError as e:
                        print("IOError")
                        time.sleep(1)
                if not success:
                    print("Failed after 10 retries!")
                                 
               
                return analog #Faz com que a função retorne o valor medido.
            
            def SensorTemperatura(sensor):
                #Escolha do Sensor de Temperatura usado.
                  if sensor==1:
                        #Sensor Termopar K.
                        Te = sensortemp.readTempC()
                        return Te
                    
                  elif sensor==8:
                        #Sensor PT100.
                        read=Read_AD(ADS.P0) #Leitura do valor analógico do canal P0 do ADS1115.
                        Vread=4.417*(read)+2.88 #Conversão do valor lido para algo entre 0 e 5 V.
                        Te=PT100X*Vread+PT100Y #Conversão do valor lido para Temperatura (ºC). 
                        
                  elif sensor==9:
                        #Sensor 4-50 mV.
                        read=Read_AD(ADS.P1) #Leitura do valor analógico do canal P1 do ADS1115.
                        Te=ffx*Vread+ffy #Conversão do valor lido para Temperatura (ºC).
                        
                  elif sensor==10:
                        #Sensor 4-20 mA.
                        read=Read_AD(ADS.P2) #Leitura do valor analógico do canal P2 do ADS1115.
                        Te=ftx*Vread+fty #Conversão do valor lido para Temperatura (ºC).
                        
                  elif sensor==11:
                        #Sensor 4-50 mV.
                        read=Read_AD(ADS.P3) #Leitura do valor analógico do canal P3 do ADS1115.
                        Te=fox*Vread+foy #Conversão do valor lido para Temperatura (ºC).
                                
            
            
            Sensor = client.get_node("ns=4;s=sensor;datatype=Float")
            sensor = round(Sensor.get_value())
            print("O sensor para ler o sinal de de entrada:", end = " ")
            print(sensor)    
            
            Saida = client.get_node("ns=4;s=saida; datatype=Float")
            saida = round(Saida.get_value())
            print("A Saída que será usada é:", end = " ")
            print(saida)            
                      
            Kp = client.get_node("ns=4;s=kp;datatype=Float")
            kp = Kp.get_value()
            print("O Kp atual é:", end = " ")
            print(kp)
            
            Ki = client.get_node("ns=4;s=ki;datatype=Float")
            ki = Ki.get_value()
            print("O Ki atual é:", end = " ")
            print(ki)
            
            Kd = client.get_node("ns=4;s=kd;datatype=Float")
            kd = Kd.get_value()
            print("O Kd atual é:", end = " ")
            print(kd)
                   
            Sp = client.get_node("ns=4;s=sp;datatype=Float")
            sp = Sp.get_value()
            print("O Setpoint atual é:", end = " ")
            print(sp)
            
            Alarme1 = client.get_node("ns=4;s=alarme1;datatype=Float")
            alarme1 = Alarme1.get_value()
            print("O Alarme1 atual é:", end = " ")
            print(alarme1)
            
            Alarme2 = client.get_node("ns=4;s=alarme2;datatype=Float")
            alarme2 = Alarme2.get_value()
            print("O Alarme2 atual é:", end = " ")
            print(alarme2)
            
            Alarme3 = client.get_node("ns=4;s=alarme3;datatype=Float")
            alarme3 = Alarme3.get_value()
            print("O Alarme3 atual é:", end = " ")
            print(alarme3)
            
            act1 = client.get_node("ns=4;s=ALARME1;datatype=Boolean") #Acionamento do alarme1.
            Act1 = act1.get_value()
            print("O acionamento do Alarme1 atual é:", end = " ")
            print(Act1)            
            
            act2 = client.get_node("ns=4;s=ALARME2;datatype=Boolean") #Acionamento do alarme2.
            Act2 = act2.get_value()
            print("O acionamento do Alarme2 atual é:", end = " ")
            print(Act2)            
            
            act3 = client.get_node("ns=4;s=ALARME3;datatype=Boolean") #Acionamento do alarme1.
            Act3 = act3.get_value()
            print("O acionamento do Alarme3 atual é:", end = " ")
            print(Act3)
            
            Merr = client.get_node("ns=4;s=merr;datatype=Float")
            merr = Merr.get_value()
            print("O merr atual é:", end = " ")
            print(merr)
            
            Igma = client.get_node("ns=4;s=igma;datatype=Boolean")
            igma = Igma.get_value()
            print("O Igma atual é:", end = " ")
            print(igma)
            
            Igme = client.get_node("ns=4;s=igme;datatype=Boolean")
            igme = Igme.get_value()
            print("O Igme atual é:", end = " ")
            print(igme)
            
            Senha = client.get_node("ns=4;s=senha;datatype=String")
            senha = Senha.get_value()
            print("A Senha atual é:", end = " ")
            print(senha)
            
            Senhan = client.get_node("ns=4;s=senhan;datatype=String")
            senhan = Senhan.get_value()
            print("A Senha Nova atual é:", end = " ")
            print(senhan)
            
            Sampletime = client.get_node("ns=4;s=sampletime;datatype=Float")
            sampletime = Sampletime.get_value()
            print("O Sampletime atual é:", end = " ")
            print(sampletime)      
            
            Controle = client.get_node("ns=4;s=control;datatype=Boolean")
            controle  = Controle.get_value()
            print("Controle  atual é:", end = " ")
            print(controle) 
            
            Trele = client.get_node("ns=4;s=trele;datatype=Float")
            trele  = Trele.get_value()
            print("O tempo do ciclo (s) do Relé é:", end = " ")
            print(trele)
            
            Freq = client.get_node("ns=4;s=freq;datatype=Float")
            freq  = Freq.get_value()
            print("A frequência do PWM é:", end = " ")
            print(freq)
            
            Resist = client.get_node("ns=4;s=resist;datatype=Float")
            resist  = Resist.get_value()
            print("A Resistência é:", end = " ")
            print(resist)
            
            Ffx = client.get_node("ns=4;s=ffx;datatype=Float")
            ffx  = Ffx.get_value()
            print("ffx é:", end = " ")
            print(ffx)
            
            Ffy = client.get_node("ns=4;s=ffy;datatype=Float")
            ffy  = Ffy.get_value()
            print("ffy é:", end = " ")
            print(ffy)
            
            Ftx = client.get_node("ns=4;s=ftx;datatype=Float")
            ftx  = Ftx.get_value()
            print("ftx é:", end = " ")
            print(ftx)
            
            Fty = client.get_node("ns=4;s=fty;datatype=Float")
            fty  = Fty.get_value()
            print("fty é:", end = " ")
            print(fty)
            
            Fox = client.get_node("ns=4;s=fox;datatype=Float")
            fox  = Fox.get_value()
            print("fox é:", end = " ")
            print(fox)
            
            Foy = client.get_node("ns=4;s=foy;datatype=Float")
            foy  = Foy.get_value()
            print("foy é:", end = " ")
            print(foy)
            
            Ftox = client.get_node("ns=4;s=ftox;datatype=Float")
            ftox  = Ftox.get_value()
            print("ftox é:", end = " ")
            print(ftox)
            
            Ftoy = client.get_node("ns=4;s=ftoy;datatype=Float")
            ftoy  = Ftoy.get_value()
            print("ftoy é:", end = " ")
            print(ftoy)
            
            #Te = client.get_node("ns=4;s=temp;datatype=Float")            
            #temp = Te.get_value()
            #Te = sensortemp.readTempC() #Leitura do sensor de temperatura.
            #Te=25
            Te = SensorTemperatura(sensor) #Leitura do sensor de temperatura.
            cliente.publish("Temperatura", Te) #Atualiza valor da temperatura no Node-RED para o lido pelo sensor por comunicação mqtt.
            print("A Temperatura é de aproximadamente:", end = " ")
            print(Te)
            

            
            time.sleep(sampletime)
            #controle= False
            setpoint = sp #Armazenamento do setpoint informado pelo usuário.
            diff=0 #Inicialização do erro do ciclo anterior do PID.
            difff=0 #Inicialização da soma dos erros dos ciclos anteriores do PID.
            out=0 #Inicialização da saída do PID.
            
            if controle == True:
            #Inicialização das configurações de GPIO da saída qeu será ultilizada no controle.
                if(saida==0):
                    #Saída Relé.
                    o=saidarele
                    GPIO.setup(o, GPIO.OUT)
                    
                elif(saida==1):
                    #Saída PWM.
                    o=saidapwm
                    GPIO.setup(o, GPIO.OUT)
                    pass
                        
                elif(saida==2):
                    #Saída 4-20mA.
                    o=saida420
                    GPIO.setup(o, GPIO.OUT)
                    pass
                    
                else:
                    print("Erro para determinar saída a ser usada")
                    
            
            while controle == True:
                  #Inicialização do PWM, caso for ultilizado:
                  if(saida==1):
                        GPIO.setup(o, GPIO.OUT)
                        GPIO.output(o, GPIO.HIGH)
                        print("pwm")
                        DT=out/100 #Duty Cycle.
                        pwm = GPIO.PWM(o, f)
                        pwm.start(0)
                        #pwm.ChangeDutyCycle(50)
                                   
                  elif(saida==2):
                        GPIO.setup(o, GPIO.OUT)
                        GPIO.output(o, GPIO.HIGH)
                        print("pwm")
                        DT=out/100 #Duty Cycle.
                        pwm = GPIO.PWM(o, f)
                        pwm.start(0)
                        #pwm.ChangeDutyCycle(50)
                  
                
                def output(out):
                    if(saida==0):
                        #Saída Relé.                        
                        GPIO.output(o, GPIO.HIGH)
                        if(out<50):
                              out=0 #Para out<50, o relé será desligado.
                        else:
                        out=100 #Para out>50, o relé será ligado.
                        ton=(out/100)*trele #Tempo GPIO ligada.
                        time.sleep(ton)
                        GPIO.output(o, GPIO.LOW)
                        toff=trele-ton #Tempo GPIO ligada
                        time.sleep(toff)
                        
                    elif(saida==1):
                        #Saída PWM.                                                                      
                        pwm.ChangeDutyCycle(out)
                        
                    elif(saida==2):
                        #Saída 4-20 mA.
                        pwm.ChangeDutyCycle(out)
                        pass
                        
                    else:
                        print("Erro para determinar saída a ser usada")

                        
                Te = SensorTemperatura(sensor) #Leitura do sensor de temperatura.                            
                cliente.publish("Temperatura", Te) #Atualiza valor da temperatura no Node-RED para o lido pelo sensor por comunicação mqtt.
                dif=setpoint-Te #Diferença da temperatura desejada para medida.
                out=out+dif*kp+diff*kd+difff*ki #Cálculo da saída do PID.
                diff=dif #Armazenamento do erro anterior.
                difff=difff+dif #Armazenamento da soma dos erros anteriores.
                
                #Para forçar com que a saída que será a porcentagem do tempo em que a GPIO ficará acionada (TOn) fique entre 0% e 100%:
                if out>100:
                    out=100
                elif out<0:
                    out=0
                
                output(out) #Saída conforme a porcentagem definida.
                #Encerra o PWM caso tenha sido ultilizado:
                if(saida==1):
                      pwm.stop
                elif(saida==2):
                      pwm.stop  
                time.sleep(sampletime)
                controle  = Controle.get_value() #Atualização da variável de controle, para que se encerre a operação quando desejado pelo usuário.
                if controle == False:
                    GPIO.cleanup() #Reseta configurações da GPIO.
            if zerocounter==1:
                  counter=0 #Zera o contador, caso o código tenha voltado a funcionar.
                  zerocounter=0
               

      except:
            counter=counter+1
            zerocounter=1    
            time.sleep(20) #Só tenta conectar com o servidor após 20 segundos.
            print("Trying reconnect to server...")
            print("Counter:")
            print(counter)
                
print("Execução do Código do Controle de Temperatura Encerrada")
           
#cd ..
    #//echo Sistema Inicializando... Agurade Alguns Instantes...
    #sudo systemctl stop nodered #Encerra o servidor Node-Red.
    #echo Node-Red encerrado
    #sleep 10 #Caso o procedimento inicial não tenha dado certo será feita uma nova tentativa com um tempo maior para iniciar o Node-Red.
    #sudo systemctl start nodered #Inicia servidor Node-Red.
    #echo Node-Red inicializado
    #cd ..
    #//echo Sistema Inicializando... Agurade Alguns Instantes...
    #sudo systemctl stop nodered #Encerra o servidor Node-Red.
    #echo Node-Red encerrado
    #sleep 10 #Caso o procedimento inicial não tenha dado certo será feita uma nova tentativa com um tempo maior para iniciar o Node-Red.
    #sudo systemctl start nodered #Inicia servidor Node-Red.
    #echo Node-Red inicializado
