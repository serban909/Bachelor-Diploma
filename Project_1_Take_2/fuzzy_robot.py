#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
import socket

TASK = "MazeSolver"   # "LaneKeeping" / "MazeSolver"
ALGORITHM = "PID"  # "PID" / "FuzzyPI" / "FuzzyAutomata" / "AbsoluteFuzzy"

def initializeConfiguration(task, algorithm):
    hardware={
        "LeftMotor": Port.A,
        "RightMotor": Port.C,
        "WheelDiameter": 55.5,
        "AxleTrack": 104,
    }
    
    wifi={
        "EnableStreaming": True,
        "PC_IP": "10.194.244.90",
        "PC_PORT": 5005,
    }
    
    if task == "LaneKeeping":
        taskConfiguration={
            "LeftColorSensor": Port.S2,
            "RightColorSensor": Port.S3,
            "BlackThreshold": 6,
            "CrossingThreshold": 7,
            "MinDifference": 3,
            "BaseSpeed": 50,
            "TurningSpeed": 25,
            "EmergencySpeed": 20,
            "EmergencyTurnSpeed": 100,
            "StopConfirmCount": 1,
            "LogInterval": 1,
        }
    elif task == "MazeSolver":
        taskConfiguration={
            "ForwardUltrasonicSensor": Port.S4,
            "LeftUltrasonicSensor": Port.S1,
            "DistanceThresholdLow": 30,
            "DistanceThresholdHigh": 50,
            "TargetWallDistance": 35,
            "BaseSpeed": 25,
            "SlowSpeed": 12.5,
            "MinForwardDistance": 60,
            "ObstacleThreshold": 30,
            "OpenPassageThreshold": 90,
            "PeekAngle": 30,
            "LogInterval": 1,
        }
    else:
        raise ValueError("Unknown task: "+task)
        
    if algorithm == "PID":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "Kp": 0.8,
                "Ki": 0.05,
                "Kd": 0.003,
                "MinOutput": -100,
                "MaxOutput": 100,
                "IntegralMin": -100.0,
                "IntegralMax": 100.0,
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "Kp": 10.0,
                "Ki": 0.01,
                "Kd": 0.02,
                "MinOutput": -100,
                "MaxOutput": 100,
                "IntegralMin": -1000.0,
                "IntegralMax": 1000.0,
            }
    elif algorithm == "FuzzyPI":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "BaseKP": 0.8,
                "BaseKI": 0.05,
                "SmallDisturbance": 3.0,
                "MediumDisturbance": 6.0,
                "LargeDisturbance": 10.0,
                "FuzzyMinOutput": -100,
                "FuzzyMaxOutput": 100,
                "FuzzyIntegralMin": -100.0,
                "FuzzyIntegralMax": 100.0,
                "Rules": [(0.8, 0.7), (1.0, 1.0), (1.2, 1.3)],  
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "BaseKP": 10.0,
                "BaseKI": 0.01,
                "SmallDisturbance": 5.0,
                "MediumDisturbance": 15.0,
                "LargeDisturbance": 30.0,
                "FuzzyMinOutput": -100,
                "FuzzyMaxOutput": 100,
                "FuzzyIntegralMin": -100.0,
                "FuzzyIntegralMax": 100.0,
                "Rules": [(0.8, 0.7), (1.0, 1.0), (1.2, 1.3)],  
            }
    elif algorithm == "FuzzyAutomata":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "SmallDisturbance": 3.0,
                "MediumDisturbance": 10.0,
                "LargeDisturbance": 20.0,
                "MinNormalization": 0.05,
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "SmallDisturbance": 5.0,
                "MediumDisturbance": 15.0,
                "LargeDisturbance": 30.0,
                "MaxTurnCorrection": 60.0,
                "MinNormalization": 0.05,
            }
    elif algorithm == "AbsoluteFuzzy":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "SmallDisturbance": 3.0,
                "MediumDisturbance": 6.0,
                "LargeDisturbance": 10.0,
                "DeltaThreshold": 3.0,
                "SmallOutput": 25.0,
                "MediumOutput": 50.0,
                "LargeOutput": 80.0,
                "FuzzyMinOutput": -100,
                "FuzzyMaxOutput": 100,
                
                # disturbance indices:  0=NL  1=NS  2=ZE  3=PS  4=PL
                # delta indices: 0=N   1=Z   2=P
                # output indices: 0=NL  1=NM  2=NS  3=ZE  4=PS  5=PM  6=PL
                
                "Rules": [
                        (0, 0, 6),  # R1:  NL & N  -> PL
                        (0, 1, 6),  # R2:  NL & Z  -> PL
                        (0, 2, 5),  # R3:  NL & P  -> PM
                        (1, 0, 5),  # R4:  NS & N  -> PM
                        (1, 1, 5),  # R5:  NS & Z  -> PM
                        (1, 2, 4),  # R6:  NS & P  -> PS
                        (2, 0, 3),  # R7:  ZE & N  -> ZE
                        (2, 1, 3),  # R8:  ZE & Z  -> ZE
                        (2, 2, 3),  # R9:  ZE & P  -> ZE
                        (3, 0, 2),  # R10: PS & N  -> NS
                        (3, 1, 1),  # R11: PS & Z  -> NM
                        (3, 2, 1),  # R12: PS & P  -> NM
                        (4, 0, 1),  # R13: PL & N  -> NM
                        (4, 1, 0),  # R14: PL & Z  -> NL
                        (4, 2, 0),  # R15: PL & P  -> NL
                    ],
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "SmallDisturbance": 3.0,
                "MediumDisturbance": 6.0,
                "LargeDisturbance": 10.0,
                "DeltaThreshold": 3.0,
                "SmallOutput": 25.0,
                "MediumOutput": 50.0,
                "LargeOutput": 80.0,
                "FuzzyMinOutput": -100,
                "FuzzyMaxOutput": 100,
            }
    else:
        raise ValueError("Unknown algorithm: "+algorithm)
    
    configuration={}
    configuration.update(hardware)
    configuration.update(wifi)
    configuration.update(taskConfiguration)
    configuration.update(algorithmConfiguration)
     
    return configuration

class WifiStreamer:
    def __init__(self, configuration, label):
        self.ip=configuration["PC_IP"]
        self.port=configuration["PC_PORT"]
        self.label=label
        self.socket=None
        
        if configuration["EnableStreaming"]:
            try:
                self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sendMeta()
                print("Wifi streaming enabled-sending to " + self.ip + ":" + str(self.port))
            except Exception as error:
                print("Warning: Could not create socket-" + str(error))
                self.socket=None
                
    def sendMeta(self):
        meta="META,"+self.label+"\n" 
        self.socket.sendto(meta.encode(), (self.ip, self.port))
        
    def send(self, step, disturbance, output): 
        if self.socket is None:
            return
        try:
            packet=str(step)+","+str(disturbance)+","+str(output)+"\n"
            self.socket.sendto(packet.encode(), (self.ip, self.port))
        except Exception as error:
            print("Warning: Could not send data-" + str(error))
            
    def close(self):
        if self.socket is not None:
            self.socket.close()
            
class ControlStrategy:
    name = "unknown"
    
    def reset(self):
        raise NotImplementedError
            
class Algorithm(ControlStrategy):
    def compute(self, setpoint, measuredpoint, dt):
        raise NotImplementedError
    
    def reset(self):
        raise NotImplementedError
    
class PIDAlgorithm(Algorithm):
    def __init__(self, configuration):
        self.kp=configuration["Kp"]
        self.ki=configuration["Ki"]
        self.kd=configuration["Kd"]
        self.minOutput=configuration["MinOutput"]
        self.maxOutput=configuration["MaxOutput"]
        self.integralMin=configuration["IntegralMin"]
        self.integralMax=configuration["IntegralMax"]
        self.integral=0.0
        self.previousDisturbance=0.0
        
    def compute(self, setPoint, measuredPoint, dt):
        disturbance=setPoint-measuredPoint
        pTerm=self.kp*disturbance
        
        if dt>0:
            self.integral+=disturbance*dt
            
            if self.integral>self.integralMax:
                self.integral=self.integralMax
            elif self.integral<self.integralMin:
                self.integral=self.integralMin
        iTerm=self.ki*self.integral
        
        if dt>0:
            derivative=(disturbance-self.previousDisturbance)/dt
        else:
            derivative=0.0
            
        dTerm=self.kd*derivative
        
        output=pTerm+iTerm+dTerm
        
        if output>self.maxOutput:
            output=self.maxOutput
        elif output<self.minOutput:
            output=self.minOutput
            
        self.previousDisturbance=disturbance
        
        return disturbance, output

    def reset(self):
        self.integral=0.0
        self.previousDisturbance=0.0
        
class FuzzyPIAlgorithm(Algorithm):
    def __init__(self, configuration):
        self.baseKp=configuration["BaseKP"]
        self.baseKi=configuration["BaseKI"]
        self.smallDisturbance=configuration["SmallDisturbance"]
        self.mediumDisturbance=configuration["MediumDisturbance"]
        self.largeDisturbance=configuration["LargeDisturbance"]
        self.fuzzyMinOutput=configuration["FuzzyMinOutput"]
        self.fuzzyMaxOutput=configuration["FuzzyMaxOutput"]
        self.fuzzyIntegralMin=configuration["FuzzyIntegralMin"]
        self.fuzzyIntegralMax=configuration["FuzzyIntegralMax"]
        self.rules=configuration["Rules"]  
        self.integral=0.0
        
    def fuzzifyDisturbance(self, disturbance):
        disturbanceAbs=abs(disturbance)
        
        if disturbanceAbs<=self.smallDisturbance:
            muSmall=1.0
        elif disturbanceAbs<=self.mediumDisturbance:
            muSmall=(self.mediumDisturbance-disturbanceAbs)/(self.mediumDisturbance-self.smallDisturbance)
        else:
            muSmall=0.0
            
        if disturbanceAbs<=self.smallDisturbance:
            muMedium=0.0
        elif disturbanceAbs<=self.mediumDisturbance:
            muMedium=(disturbanceAbs-self.smallDisturbance)/(self.mediumDisturbance-self.smallDisturbance)
        elif disturbanceAbs<=self.largeDisturbance:
            muMedium=(self.largeDisturbance-disturbanceAbs)/(self.largeDisturbance-self.mediumDisturbance)
        else:
            muMedium=0.0
            
        if disturbanceAbs<=self.mediumDisturbance:
            muLarge=0.0
        elif disturbanceAbs<=self.largeDisturbance:
            muLarge=(disturbanceAbs-self.mediumDisturbance)/(self.largeDisturbance-self.mediumDisturbance)
        else:
            muLarge=1.0
            
        return muSmall, muMedium, muLarge
    
    def inference(self, muSmall, muMedium, muLarge):
        kpSmall = self.rules[0][0]
        kiSmall = self.rules[0][1]
        
        kpMedium = self.rules[1][0]
        kiMedium = self.rules[1][1]
        
        kpLarge = self.rules[2][0]
        kiLarge = self.rules[2][1]
        
        totalWeight = muSmall + muMedium + muLarge
        
        if totalWeight > 0:
            kpMultiplier = (muSmall * kpSmall + muMedium * kpMedium + muLarge * kpLarge) / totalWeight
            kiMultiplier = (muSmall * kiSmall + muMedium * kiMedium + muLarge * kiLarge) / totalWeight
        else:
            kpMultiplier = 1.0
            kiMultiplier = 1.0
            
        kpAdaptive = self.baseKp * kpMultiplier
        kiAdaptive = self.baseKi * kiMultiplier

        return kpAdaptive, kiAdaptive
    
    def compute(self, setPoint, measuredPoint, dt):
        disturbance=setPoint-measuredPoint
        muSmall, muMedium, muLarge=self.fuzzifyDisturbance(disturbance)
        kpAdaptive, kiAdaptive=self.inference(muSmall, muMedium, muLarge)
        
        pTerm=kpAdaptive*disturbance
        
        if dt>0:
            self.integral+=disturbance*dt
            
            if self.integral>self.fuzzyIntegralMax:
                self.integral=self.fuzzyIntegralMax
            elif self.integral<self.fuzzyIntegralMin:
                self.integral=self.fuzzyIntegralMin
                
        iTerm=kiAdaptive*self.integral
        
        output=pTerm+iTerm
        
        if output>self.fuzzyMaxOutput:
            output=self.fuzzyMaxOutput
        elif output<self.fuzzyMinOutput:
            output=self.fuzzyMinOutput
            
        return disturbance, output
    
    def reset(self):
        self.integral=0.0
        
class AbsoluteFuzzy(Algorithm):
    def __init__(self, configuration):
        self.smallDisturbance=configuration["SmallDisturbance"]
        self.mediumDisturbance=configuration["MediumDisturbance"]
        self.largeDisturbance=configuration["LargeDisturbance"]
        self.deltaThreshold=configuration["DeltaThreshold"]
        self.smallOutput=configuration["SmallOutput"]
        self.mediumOutput=configuration["MediumOutput"]
        self.largeOutput=configuration["LargeOutput"]
        self.fuzzyMinOutput=configuration["FuzzyMinOutput"]
        self.fuzzyMaxOutput=configuration["FuzzyMaxOutput"]
        self.rulesTable=configuration["Rules"]
        
        self.previousDisturbance=0.0
        self.firstCall = True
        self.lastActiveRules=[]
        
        self.outputSingletons=[
            -self.largeOutput,
            -self.mediumOutput,
            -self.smallOutput,
            0.0,
            self.smallOutput,
            self.mediumOutput,
            self.largeOutput,
        ]
        
    def clamp(self, value):
        if value<0:
            return 0.0
        elif value>1:
            return 1.0
        return value
        
    def fuzzifyDisturbance(self, disturbance):
        S=self.smallDisturbance
        M=self.mediumDisturbance
        
        muNL=self.clamp((-S-disturbance)/(M-S))
        
        if disturbance<=-M or disturbance>=0.0:
            muNS=0.0
        elif disturbance<=-S:
            muNS=(disturbance+M)/(M-S)
        else:
            muNS=(-disturbance)/S
            
        if disturbance<=-S or disturbance>=S:
            muZE=0.0
        elif disturbance<0.0:
            muZE=(disturbance+S)/S
        else:
            muZE=(S-disturbance)/S
            
        if disturbance<=0.0 or disturbance>=M:
            muPS=0.0
        elif disturbance<=S:
            muPS=disturbance/S
        else:
            muPS=(M-disturbance)/(M-S)
            
        muPL=self.clamp((disturbance-S)/(M-S))
        
        return (muNL, muNS, muZE, muPS, muPL)
        
    def fuzzifyDelta(self, delta):
        D=self.deltaThreshold
        
        muN=self.clamp((-delta)/D)
        
        if delta<=-D or delta>=D:
            muZ=0.0
        elif delta<0.0:
            muZ=(delta+D)/D
        else:
            muZ=(D-delta)/D    
            
        muP=self.clamp(delta/D)
        
        return (muN, muZ, muP)
    
    def updateDelta(self, disturbance):
        if self.firstCall:
            self.previousDisturbance = disturbance
            self.firstCall = False
        delta = disturbance - self.previousDisturbance
        self.previousDisturbance = disturbance
        
        return delta
    
    def inference(self, muDisturbance, muDelta):
        aggregation=[0.0]*len(self.outputSingletons)

        for disturbanceIndex, deltaIndex, outputIndex in self.rulesTable:
            ruleStrength=min(muDisturbance[disturbanceIndex], muDelta[deltaIndex])

            if ruleStrength>0.0:
                self.lastActiveRules.append((disturbanceIndex, deltaIndex, outputIndex, ruleStrength))

                if ruleStrength>aggregation[outputIndex]:
                    aggregation[outputIndex]=ruleStrength

        return aggregation
    
    def defuzzify(self, aggregation):
        numerator=0.0
        denominator=0.0
        
        for i in range(len(self.outputSingletons)):
            numerator+=aggregation[i]*self.outputSingletons[i]
            denominator+=aggregation[i]
            
        if denominator>0.0:
            return numerator/denominator
        else:
            return 0.0
        
    def logRules(self):
        disturbanceLabels=("NL", "NS", "ZE", "PS", "PL")
        deltaLabels=("N", "Z", "P")
        outputLabels=("NL", "NM", "NS", "ZE", "PS", "PM", "PL")
        
        if not self.lastActiveRules:
            return "No active rules"
        
        parts=[]
        
        for disturbanceIndex, deltaIndex, outputIndex, strength in self.lastActiveRules:
            parts.append(disturbanceLabels[disturbanceIndex]+" & "+deltaLabels[deltaIndex]+" -> "+outputLabels[outputIndex]+": "+str(round(strength, 2)))
            
        return "Rules("+str(len(self.lastActiveRules))+") "+", ".join(parts)
        
    def compute(self, setPoint, measuredPoint, dt):
        self.lastActiveRules=[]
        disturbance=setPoint-measuredPoint
        delta=self.updateDelta(disturbance)
        
        muDisturbance=self.fuzzifyDisturbance(disturbance)
        muDelta=self.fuzzifyDelta(delta)
        
        aggregation=self.inference(muDisturbance, muDelta)
        
        output=self.defuzzify(aggregation)
        
        if output>self.fuzzyMaxOutput:
            output=self.fuzzyMaxOutput
        elif output<self.fuzzyMinOutput:
            output=self.fuzzyMinOutput
        
        return disturbance, output
        
    def reset(self):
        self.previousDisturbance=0.0
        self.firstCall = True
        self.lastActiveRules=[]
        self.previousActiveRules=[]
           
class SensorData:
    def __init__(self, configuration, **kwargs):
        self.configuration=configuration
        for key, value in kwargs.items():
            setattr(self, key, value)
                
class FuzzyState:
    def __init__(self, name, initialMV=0.0):
        self.name=name
        self.mv=initialMV
        
    def getOutput(self, configuration):
        raise NotImplementedError
        
class FuzzyTransition:
    def __init__(self, fromState, toState, configuration):
        self.fromState=fromState
        self.toState=toState
        self.turningSpeed=configuration["TurningSpeed"]
    def evaluateDisturbance(self, sensorData):
        raise NotImplementedError
    
class FuzzyStateMachine(Algorithm):
    name = "FuzzyAutomata"
    
    def __init__(self, states, transitions, configuration):
        self.states=states
        self.transitions=transitions
        self.configuration=configuration
        self.initialMVs=[state.mv for state in states]
        self.normFloor=configuration.get("MinNormalization", 0.05)
        
    def cacheConditions(self, sensorData):
        return {id(transition): transition.evaluateDisturbance(sensorData) for transition in self.transitions}
    
    def computeNextMV(self, sensorData):
        cache=self.cacheConditions(sensorData)
        rawMVs={}
        
        for state in self.states:
            muIn=0.0
            
            for transition in self.transitions:
                if transition.toState==state:
                    contribution=min(transition.fromState.mv, cache[id(transition)])
                    if contribution>muIn:
                        muIn=contribution
                        
            maxOut=0.0
            
            for transition in self.transitions:
                if transition.fromState==state:
                    value=cache[id(transition)]
                    if value>maxOut:
                        maxOut=value
            muStay=1.0-maxOut
            muPersist=min(state.mv, muStay)
            
            rawMVs[state]=max(muIn, muPersist)
            
        maxRaw=max(rawMVs.values())
        
        if maxRaw>=self.normFloor:
            for state in self.states:
                state.mv=rawMVs[state]/maxRaw
        else:
            for state in self.states:
                state.mv=rawMVs[state]
                
    def defuzzify(self):
        totalSum=sum(state.mv for state in self.states)
        
        if totalSum==0.0:
            return self.states[0].getOutput(self.configuration)
        
        speedAcc=0.0
        turnAcc=0.0
        
        for state in self.states:
            speed, turn =state.getOutput(self.configuration)
            speedAcc+=speed*state.mv
            turnAcc+=turn*state.mv
            
        return speedAcc/totalSum, turnAcc/totalSum    
    
    def tick(self, sensorData):
        self.computeNextMV(sensorData)
        return self.defuzzify()
    
    def reset(self):
        for state, initialMV in zip(self.states, self.initialMVs):
            state.mv=initialMV
            
    def logMVs(self):
        return "  ".join(s.name + ":" + str(round(s.mv, 2)) for s in self.states)
               
def hardware_setup(configuration):
    ev3=EV3Brick()
    leftMotor=Motor(configuration["LeftMotor"])
    rightMotor=Motor(configuration["RightMotor"])
    robot=DriveBase(leftMotor, rightMotor, configuration["WheelDiameter"], configuration["AxleTrack"])
    
    return ev3, robot

class Controller:
    def __init__(self, algorithm, configuration, streamer, ev3, robot):
        self.algorithm=algorithm
        self.configuration=configuration
        self.streamer=streamer
        self.ev3=ev3
        self.robot=robot
        self.stopwatch=StopWatch()
        self.lastLogTime=0.0
        self.step=0
        
    def run(self):
        raise NotImplementedError
    
    def now(self):
        return self.stopwatch.time() / 1000.0
    
    def nextStep(self):
        self.step+=1
        return self.step, self.now()
    
class LaneKeepingController(Controller):
    def __init__(self, algorithm, configuration, streamer, ev3, robot):
        super().__init__(algorithm, configuration, streamer, ev3, robot)
        self.leftSensor=ColorSensor(configuration["LeftColorSensor"])
        self.rightSensor=ColorSensor(configuration["RightColorSensor"])
        self.blackThreshold=configuration["BlackThreshold"]
        self.crossingThreshold=configuration["CrossingThreshold"]
        self.minDifference=configuration["MinDifference"]
        self.baseSpeed=configuration["BaseSpeed"]
        self.turningSpeed=configuration["TurningSpeed"]
        self.emergencySpeed=configuration["EmergencySpeed"]
        self.emergencyTurnSpeed=configuration["EmergencyTurnSpeed"]
        self.stopConfirmCount=configuration["StopConfirmCount"]
        self.logInterval=configuration["LogInterval"]
        
    def readSensors(self):
        leftValue=self.leftSensor.reflection()
        rightValue=self.rightSensor.reflection()
        return leftValue, rightValue
    
    def printHeader(self, algorithm):
        self.ev3.speaker.beep()
        print("\nLANE KEEPING-"+algorithm+"\n")
        print("Black Threshold:"+ str(self.blackThreshold))
        print("Crossing Threshold:"+ str(self.crossingThreshold))
        print("Min Difference:"+ str(self.minDifference))
        print("Base Speed:"+ str(self.baseSpeed))
        print("Turning Speed:"+ str(self.turningSpeed))
        print("Emergency Speed:"+ str(self.emergencySpeed)+"\n")
        
    def decideAction(self, leftValue, rightValue, difference, dt):
        if leftValue<self.crossingThreshold and rightValue>=self.crossingThreshold:
            return self.emergencySpeed, self.emergencyTurnSpeed
        
        if rightValue<self.crossingThreshold and leftValue>=self.crossingThreshold:
            return self.emergencySpeed, -self.emergencyTurnSpeed
        
        if abs(difference)<=self.minDifference:
            return self.baseSpeed, 0
        
        disturbance, turnRate=self.algorithm.compute(0, difference, dt)
        self.streamer.send(self.step, disturbance, turnRate)
        
        return self.turningSpeed, turnRate
    
    def run(self):
        if isinstance(self.algorithm, FuzzyPIAlgorithm):    
            algorithm="FuzzyPI"
        elif isinstance(self.algorithm, AbsoluteFuzzy):
            algorithm="AbsoluteFuzzy"
        elif isinstance(self.algorithm, FuzzyStateMachine):
            algorithm="FuzzyAutomata"
        else:
            algorithm="PID"
        
        self.printHeader(algorithm)
        
        bothSensorsOnLineCount=0
        lastTime=0.0
        
        try:
            while True:
                step, currentTime=self.nextStep()
                dt=currentTime-lastTime
                lastTime=currentTime
                
                leftValue, rightValue=self.readSensors()
                
                if step%self.logInterval==0:
                    print("Step "+str(step)+" Left: "+str(leftValue)+" Right: "+str(rightValue))
                    if isinstance(self.algorithm, AbsoluteFuzzy):
                        print(self.algorithm.logRules())
                
                leftOnBlack=leftValue<self.blackThreshold
                rightOnBlack=rightValue<self.blackThreshold
                
                if leftOnBlack and rightOnBlack:
                    bothSensorsOnLineCount+=1
                    if bothSensorsOnLineCount>=self.stopConfirmCount:
                        self.robot.stop()
                        self.ev3.speaker.beep()
                        print("\nBoth sensors on line for "+str(self.stopConfirmCount)+" consecutive steps. Stopping.")
                        break
                    
                    self.robot.drive(10, 0)
                    wait(20)
                    continue
                else:
                    bothSensorsOnLineCount=0
                    
                difference=leftValue-rightValue
                speed, turnRate=self.decideAction(leftValue, rightValue, difference, dt)
                
                if speed<10:
                    speed=10
                    
                self.robot.drive(speed, turnRate)
                wait(20)
                
        except KeyboardInterrupt:
            self.robot.stop()
            self.ev3.speaker.beep()
            print("\nLANE KEEPING STOPPED\n")
        finally:
            self.streamer.close()
            
class MazeSolverController(Controller):
    def __init__(self, algorithm, configuration, streamer, ev3, robot):
        super().__init__(algorithm, configuration, streamer, ev3, robot)
        self.ev3=ev3
        self.robot=robot
        self.forwardSensor=UltrasonicSensor(configuration["ForwardUltrasonicSensor"])
        self.leftSensor=UltrasonicSensor(configuration["LeftUltrasonicSensor"])
        self.distanceThresholdLow=configuration["DistanceThresholdLow"]
        self.distanceThresholdHigh=configuration["DistanceThresholdHigh"]
        self.targetWallDistance=configuration["TargetWallDistance"]
        self.baseSpeed=configuration["BaseSpeed"]
        self.slowSpeed=configuration["SlowSpeed"]
        self.minForwardDistance=configuration["MinForwardDistance"]
        self.obstacleThreshold=configuration["ObstacleThreshold"]
        self.openPassageThreshold=configuration["OpenPassageThreshold"]
        self.peekAngle=configuration["PeekAngle"]
        self.logInterval=configuration["LogInterval"]
        self.forwardHistory=[0, 0, 0, 0, 0]
        self.leftHistory=[0, 0, 0, 0, 0]

    def filterReadings(self, history, newReading):
        # Shift window and insert new sample
        history[0]=history[1]
        history[1]=history[2]
        history[2]=history[3]
        history[3]=history[4]
        history[4]=newReading

        # Median of 5: sorts a copy, returns the middle value.
        # Robust against single-sample cross-talk spikes (double-bounce echoes).
        s=sorted(history)
        return s[2]

    def flushSensors(self):
        wait(50)

        forwardLive=self.forwardSensor.distance()
        leftLive=self.leftSensor.distance()
        self.forwardHistory=[forwardLive, forwardLive, forwardLive, forwardLive, forwardLive]
        self.leftHistory=[leftLive, leftLive, leftLive, leftLive, leftLive]
    
    def readSensors(self):
        forwardRaw=self.forwardSensor.distance()
        leftRaw=self.leftSensor.distance()
        
        forwardFiltered=self.filterReadings(self.forwardHistory, forwardRaw)
        leftFiltered=self.filterReadings(self.leftHistory, leftRaw)
        
        return forwardFiltered, leftFiltered
    
    def printHeader(self, algorithm):
        self.ev3.speaker.beep()
        print("\nMAZE SOLVER-"+algorithm+"\n")
        print("Target Wall Distance:"+ str(self.targetWallDistance))
        print("Base Speed:"+ str(self.baseSpeed))
        print("Obstacle Threshold:"+ str(self.obstacleThreshold))
        
    def peekAndTurn(self, targetAngle, reason):
        print("Peek and Turn: "+reason)
        
        if targetAngle>0:
            stepAngle=self.peekAngle
        else:
            stepAngle=-self.peekAngle
        
        totalTurned=0
        turnsMade=0
        
        while abs(totalTurned)<abs(targetAngle):
            self.robot.turn(stepAngle)
            totalTurned+=stepAngle
            turnsMade+=1
            self.flushSensors()
            
            print("Peek "+str(turnsMade)+": "+str(abs(totalTurned))+"° turned")
            
            forward, left=self.readSensors()
            print("Forward: "+str(int(forward))+" mm --- Left: "+str(int(left))+" mm")
            
            if forward>self.minForwardDistance:
                print("Clear Path after "+str(abs(totalTurned))+"°")
                break
            
            if abs(totalTurned)>=abs(targetAngle):
                print("Full "+str(abs(targetAngle))+"° turn complete")
                break
            
        return totalTurned
    
    def handleObstacle(self, forward, left):
        print("\n OBSTACLE: Fwd=" + str(int(forward)) + "mm --- Left=" + str(int(left)) + "mm")
        self.robot.stop()
        wait(200)
        self.algorithm.reset()
        
        forward, left=self.readSensors()
        
        if left>self.openPassageThreshold:
            print("Decision: Left CLEAR - turning left")
            self.peekAndTurn(-90, "Left passage detected")
            return
        
        if forward>self.minForwardDistance:
            print("Decision: Forward CLEAR - continuing straight")
            return
        
        print("Decision: Front AND Left blocked - trying RIGHT")
        self.robot.turn(90)
        self.flushSensors()
        
        forward, _= self.readSensors()
        
        if forward>self.minForwardDistance:
            print("Right turn successful")
            return
        else:
            print("Still blocked, turn around")
            self.robot.turn(90)
            self.flushSensors()
            
    def handleOpenPassage(self, left):
        print("\nOPEN PASSAGE: Left="+str(int(left))+" mm")
        self.robot.stop()
        wait(200)
        self.algorithm.reset()
        self.peekAndTurn(-90, "Left passage detected")
    
    def run(self):
        if isinstance(self.algorithm, FuzzyPIAlgorithm):    
            algorithm="FuzzyPI"
        elif isinstance(self.algorithm, AbsoluteFuzzy):
            algorithm="AbsoluteFuzzy"
        elif isinstance(self.algorithm, FuzzyStateMachine):
            algorithm="FuzzyAutomata"
        else:
            algorithm="PID"
        
        self.printHeader(algorithm)
        self.flushSensors()
        
        lastTime=0.0
        
        try:
            while True:
                step, currentTime=self.nextStep()
                dt=currentTime-lastTime
                lastTime=currentTime
                
                forward, left=self.readSensors()
                
                if step%self.logInterval==0:
                    print("Step "+str(step)+" Forward: "+str(int(forward))+" mm --- Left: "+str(int(left))+" mm")
                    if isinstance(self.algorithm, AbsoluteFuzzy):
                        print(self.algorithm.logRules())
                
                if forward<self.obstacleThreshold:
                    self.handleObstacle(forward, left)
                    lastTime=self.now() 
                    continue
                
                if left>self.openPassageThreshold:
                    self.handleOpenPassage(left)
                    lastTime=self.now()
                    continue
                
                if forward<self.minForwardDistance:
                    speed=self.slowSpeed
                else:
                    speed=self.baseSpeed
                disturbance, turnRate=self.algorithm.compute(self.targetWallDistance, left, dt)
                self.streamer.send(step, disturbance, turnRate)

                self.robot.drive(speed, turnRate)
                wait(20)
                
        except KeyboardInterrupt:
            self.robot.stop()
            self.ev3.speaker.beep()
            print("\nMAZE SOLVER STOPPED\n")
        finally:
            self.streamer.close()
            
def createController(task, algorithm_name):
    configuration=initializeConfiguration(task, algorithm_name)
    ev3, robot=hardware_setup(configuration)
    label=task+"-"+algorithm_name    
    streamer=WifiStreamer(configuration, label)
    
    if algorithm_name=="PID":
        algorithm=PIDAlgorithm(configuration)
    elif algorithm_name=="FuzzyPI":
        algorithm=FuzzyPIAlgorithm(configuration)
    elif algorithm_name=="AbsoluteFuzzy":
        algorithm=AbsoluteFuzzy(configuration)


    if task=="LaneKeeping":
        return LaneKeepingController(algorithm, configuration, streamer, ev3, robot)
    elif task=="MazeSolver":
        return MazeSolverController(algorithm, configuration, streamer, ev3, robot)
    else:
        raise ValueError("Unknown task: "+task)
    
def main():
    controller=createController(TASK, ALGORITHM)
    controller.run()
    
if __name__=="__main__":
    main()