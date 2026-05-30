#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
import socket

TASK = "MazeSolver"   # "LaneKeeping" / "MazeSolver"
ALGORITHM = "FuzzyRuleBased"  # "PID" / "FuzzyPI" / "FuzzyRuleBased"

def initializeConfiguration(task, algorithm):
    hardware={
        "LeftMotor": Port.A,
        "RightMotor": Port.D,
        "WheelDiameter": 55.5,
        "AxleTrack": 104,
    }
    
    wifi={
        "EnableStreaming": True,
        "PC_IP": "10.238.186.90",
        "PC_PORT": 5005,
    }
    
    if task == "LaneKeeping":
        taskConfiguration={
            "LeftColorSensor":    Port.S2,
            "RightColorSensor":   Port.S3,
            "BlackThreshold":     6,
            "CrossingThreshold":  7,
            "MinDifference":      3,
            "BaseSpeed":          90,
            "TurningSpeed":       60,
            "EmergencySpeed":     30,
            "EmergencyTurnSpeed": 45,
            "StopConfirmCount":   1,
            "LogInterval":        1,
        }
    elif task == "MazeSolver":
        taskConfiguration={
            "ForwardUltrasonicSensor": Port.S4,   
            "LeftUltrasonicSensor":    Port.S1,
            "RightUltrasonicSensor":   Port.S2,   
            "GyroSensor":              Port.S3,
            "CellDistanceNS":          278,
            "CellDistanceEW":          278,
            "OpenPassageThreshold":    90,
            "ObstacleThreshold":       30,
            "TargetForwardDistance":   60,
            "BaseSpeed":               90,
            "MinSpeed":                30,
            "HeadingThreshold":        3,
            "LogInterval":             1,
        }
    else:
        raise ValueError("Unknown task: "+task)
        
    if algorithm == "PID":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "Kp":          0.8,
                "Ki":          0.05,
                "Kd":          0.01,
                "MinOutput":   -100,
                "MaxOutput":   100,
                "IntegralMin": -500.0,
                "IntegralMax": 500.0,
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "Kp":          0.06,
                "Ki":          0.01,
                "Kd":          0.0,
                "MinOutput":   30,
                "MaxOutput":   90,
                "IntegralMin": 0.0,
                "IntegralMax": 500.0,
            }
    elif algorithm == "FuzzyPI":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "BaseKP":            0.8,
                "BaseKI":            0.05,
                "SmallDisturbance":  3.0,
                "MediumDisturbance": 6.0,
                "LargeDisturbance":  10.0,
                "FuzzyMinOutput":    -100,
                "FuzzyMaxOutput":    100,
                "FuzzyIntegralMin":  -1000.0,
                "FuzzyIntegralMax":  1000.0,
                "Rules": [
                    (0.8, 0.7), 
                    (1.0, 1.0), 
                    (1.2, 1.3)
                ],
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "BaseKP":            0.06,
                "BaseKI":            0.001,
                "SmallDisturbance":  10.0,
                "MediumDisturbance": 20.0,
                "LargeDisturbance":  30.0,
                "FuzzyMinOutput":    30,
                "FuzzyMaxOutput":    90,
                "FuzzyIntegralMin":  0.0,
                "FuzzyIntegralMax":  500.0,
                "Rules": [
                    (0.8, 0.7), 
                    (1.0, 1.0), 
                    (1.2, 1.3)
                ],
            }
    elif algorithm == "FuzzyRuleBased":
        if task == "LaneKeeping":
            algorithmConfiguration={
                "SmallDisturbance":  3.0,
                "MediumDisturbance": 6.0,
                "DeltaThreshold":    3.0,
                "SmallOutput":       5.0,
                "MediumOutput":      20.0,
                "LargeOutput":       50.0,
                
                # disturbance indices:  0=NL  1=NS  2=ZE  3=PS  4=PL
                # delta indices: 0=N   1=Z   2=P
                # output indices: 0=NL  1=NM  2=NS  3=ZE  4=PS  5=PM  6=PL
                
                "Rules": [
                        (0, 0, 5),  # R1:  NL & N  -> PM
                        (0, 1, 6),  # R2:  NL & Z  -> PL
                        (0, 2, 6),  # R3:  NL & P  -> PL
                        (1, 0, 4),  # R4:  NS & N  -> PS
                        (1, 1, 4),  # R5:  NS & Z  -> PS
                        (1, 2, 5),  # R6:  NS & P  -> PM
                        (2, 0, 3),  # R7:  ZE & N  -> ZE
                        (2, 1, 3),  # R8:  ZE & Z  -> ZE
                        (2, 2, 3),  # R9:  ZE & P  -> ZE
                        (3, 0, 2),  # R10: PS & N  -> NS
                        (3, 1, 2),  # R11: PS & Z  -> NS
                        (3, 2, 1),  # R12: PS & P  -> NM
                        (4, 0, 1),  # R13: PL & N  -> NM
                        (4, 1, 0),  # R14: PL & Z  -> NL
                        (4, 2, 0),  # R15: PL & P  -> NL
                    ],
            }
        elif task == "MazeSolver":
            algorithmConfiguration={
                "SmallDisturbance":  10.0,
                "MediumDisturbance": 20.0,
                "DeltaThreshold":    1.0,
                "SmallOutput":       30.0,
                "MediumOutput":      60.0,
                "LargeOutput":       90.0,
                "Rules": [
                        (0, 1, 6),  # R1:  NL & Z  -> PL
                        (1, 1, 6),  # R2:  NS & Z  -> PL
                        (2, 1, 5),  # R3:  ZE & Z  -> PM
                        (3, 1, 4),  # R4:  PS & Z  -> PS
                        (4, 1, 4),  # R5:  PL & Z  -> PS
                    ],
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
        
    def sendPacket(self, packet):
        if self.socket is None:
            return
        try:
            self.socket.sendto(packet.encode(), (self.ip, self.port))
        except Exception as error:
            print("Warning: Could not send data-" + str(error))

    def send(self, step, disturbance, output):
        self.sendPacket(str(step)+","+str(disturbance)+","+str(output)+"\n")

    def sendCell(self, x, y, walls):
        self.sendPacket("CELL,"+str(x)+","+str(y)+","+str(walls)+"\n")
            
    def close(self):
        if self.socket is not None:
            self.socket.close()
            
class Algorithm:
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
        
class FuzzyRuleBased(Algorithm):
    def __init__(self, configuration):
        self.smallDisturbance=configuration["SmallDisturbance"]
        self.mediumDisturbance=configuration["MediumDisturbance"]
        self.deltaThreshold=configuration["DeltaThreshold"]
        self.smallOutput=configuration["SmallOutput"]
        self.mediumOutput=configuration["MediumOutput"]
        self.largeOutput=configuration["LargeOutput"]
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
        
        return disturbance, output
        
    def reset(self):
        self.previousDisturbance=0.0
        self.firstCall=True
        self.lastActiveRules=[]
                         
class MazeMap:
    NORTH = 0
    EAST  = 1
    SOUTH = 2
    WEST  = 3

    DX      = (0,  1,  0, -1)         
    DY      = (1,  0, -1,  0)          
    TURN    = (0, 90, 180, -90)        
    DFS     = (0,  3,  1,  2)          
    BACKDIR = {(0,1):0, (1,0):1, (0,-1):2, (-1,0):3}

    def __init__(self):
        self.visited           = set()
        self.path              = []      
        self.x                 = 0
        self.y                 = 0
        self.heading           = 0       
        self.expectedGyroAngle = 0
        self.cellWalls         = {}
        self.entryDirection    = None      

    def markVisited(self):
        self.visited.add((self.x, self.y))

    def gyroTarget(self):
        return self.expectedGyroAngle

    def planMove(self, openAbsDirs):
        for step in self.DFS:
            absDir=(self.heading + step) % 4
            
            if absDir not in openAbsDirs:
                continue
            
            nx=self.x + self.DX[absDir]
            ny=self.y + self.DY[absDir]
            
            if (nx, ny) not in self.visited:
                return absDir, self.TURN[step]
            
        return None, None
    
    def planBacktrack(self):
        if not self.path:
            return None, None
        
        px, py=self.path[-1]
        backDir=self.BACKDIR[(px-self.x, py-self.y)]
        step=(backDir - self.heading) % 4
        
        return self.TURN[step], backDir  

    def applyTurn(self, absDir):
        step = (absDir - self.heading) % 4
        self.expectedGyroAngle += self.TURN[step]   
        self.heading = absDir
        
    def storeCellInfo(self, openDirs):
        if(self.x, self.y) not in self.cellWalls:
            walls=0
            
            for direction in openDirs:
                walls |= (1 << direction)
                
            if self.entryDirection is not None:
                walls |= (1 << ((self.entryDirection + 2) % 4))
                    
            self.cellWalls[(self.x, self.y)]=walls
            
            return walls
        else:
            return None            
        
    def commitMove(self):
        self.entryDirection=self.heading
        self.path.append((self.x, self.y))
        self.x+=self.DX[self.heading]
        self.y+=self.DY[self.heading]
        self.markVisited()
    
    def commitBacktrack(self):
        self.x, self.y=self.path.pop()     

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
        self.step=0
        
    def run(self):
        raise NotImplementedError
    
    def now(self):
        return self.stopwatch.time() / 1000.0
    
    def nextStep(self):
        self.step+=1
        return self.step, self.now()

    def algorithmLabel(self):
        if isinstance(self.algorithm, FuzzyPIAlgorithm):
            return "FuzzyPI"
        elif isinstance(self.algorithm, FuzzyRuleBased):
            return "FuzzyRuleBased"
        else:
            return "PID"
    
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
        
    def decideAction(self, leftValue, rightValue, disturbance, dt):
        if leftValue<self.crossingThreshold and rightValue>=self.crossingThreshold:
            return self.emergencySpeed, self.emergencyTurnSpeed
        
        if rightValue<self.crossingThreshold and leftValue>=self.crossingThreshold:
            return self.emergencySpeed, -self.emergencyTurnSpeed
        
        if abs(disturbance)<=self.minDifference and isinstance(self.algorithm, (PIDAlgorithm, FuzzyPIAlgorithm)):
            return self.baseSpeed, 0
        
        disturbance, turnRate=self.algorithm.compute(0, disturbance, dt)
        self.streamer.send(self.step, disturbance, turnRate)
        
        return self.turningSpeed, turnRate
    
    def run(self):
        self.printHeader(self.algorithmLabel())
        
        bothSensorsOnLineCount=0
        lastTime=0.0
        
        try:
            while True:
                step, currentTime=self.nextStep()
                dt=currentTime-lastTime
                lastTime=currentTime
                
                leftValue, rightValue=self.readSensors()
                
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
                    
                disturbance=leftValue-rightValue
                speed, turnRate=self.decideAction(leftValue, rightValue, disturbance, dt)
                
                if step%self.logInterval==0:
                    print("Step "+str(step)+" Left: "+str(leftValue)+" Right: "+str(rightValue)+" Output: "+str(turnRate))
                    if isinstance(self.algorithm, FuzzyRuleBased):
                        print(self.algorithm.logRules())
                
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
        self.forwardSensor      = UltrasonicSensor(configuration["ForwardUltrasonicSensor"])
        self.leftSensor         = UltrasonicSensor(configuration["LeftUltrasonicSensor"])
        self.rightSensor        = UltrasonicSensor(configuration["RightUltrasonicSensor"])
        self.gyro               = GyroSensor(configuration["GyroSensor"])
        self.cellDistanceNS     = configuration["CellDistanceNS"]
        self.cellDistanceEW     = configuration["CellDistanceEW"]
        self.openPassage            = configuration["OpenPassageThreshold"]
        self.obstacleThreshold      = configuration["ObstacleThreshold"]
        self.targetForwardDistance  = configuration["TargetForwardDistance"]
        self.baseSpeed              = configuration["BaseSpeed"]
        self.minSpeed               = configuration["MinSpeed"]
        self.headingThreshold   = configuration["HeadingThreshold"]
        self.logInterval        = configuration["LogInterval"]
        self.mazeMap            = MazeMap()

    def scanForward(self):
        r0 = self.forwardSensor.distance()
        wait(10)
        r1 = self.forwardSensor.distance()
        wait(10)
        r2 = self.forwardSensor.distance()
        r = [r0, r1, r2]
        r.sort()
        return r[1]

    def scanJunction(self):
        openDirs = set()
        h=self.mazeMap.heading
        
        if self.scanForward()>=self.openPassage:
            openDirs.add(h)
        if self.leftSensor.distance()>=self.openPassage:
            openDirs.add((h+3) % 4)
        if self.rightSensor.distance()>=self.openPassage:
            openDirs.add((h+1) % 4)
            
        return openDirs

    def gyroError(self):
        error = self.mazeMap.gyroTarget() - self.gyro.angle()
        return (error + 180) % 360 - 180

    def alignHeading(self):
        target=self.mazeMap.gyroTarget()
        error=target - self.gyro.angle()
        correction = ((error + 180) % 360) - 180
        if abs(correction) > self.headingThreshold:
            self.robot.turn(correction)
            wait(200)
            
    def centerInCorridor(self):
        leftDist = self.leftSensor.distance()
        rightDist = self.rightSensor.distance()
        
        if leftDist < self.openPassage and rightDist < self.openPassage and abs(leftDist - rightDist) > self.minSpeed:
            correction = (rightDist - leftDist) * 0.5
            
            if correction > 30:
                correction = 30
            elif correction < -30:
                correction = -30
            
            self.robot.turn(correction)
            wait(10)
                
    def cellDist(self, absDir):
        if absDir%2==0:
            return self.cellDistanceNS
        else:
            return self.cellDistanceEW
        
    def executeTurn(self, degreeTurn):
        if degreeTurn == 0:
            return
        
        if abs(degreeTurn) == 90:
            startAngle = self.gyro.angle()
            targetAngle = startAngle + degreeTurn
            deadline = self.now() + 5.0
            
            turnRate=30 if degreeTurn > 0 else -30
            self.robot.turn(turnRate)
            
            if degreeTurn > 0:
                while self.gyro.angle() < targetAngle and self.now() < deadline:
                    wait(10)
            else:
                while self.gyro.angle() > targetAngle and self.now() < deadline:
                    wait(10)
            self.robot.stop()
        else:
            self.robot.turn(degreeTurn)
        
    def driveCell(self, absDir):
        cellTarget = self.cellDist(absDir)
        travelDist = 0.0
        prevSpeed  = 0.0
        lastTime   = self.now()
        self.algorithm.reset()
        wait(20)
        
        forward = self.scanForward()

        while travelDist < cellTarget and forward >= self.obstacleThreshold:
            currentTime = self.now()
            dt          = currentTime - lastTime
            lastTime    = currentTime

            forward = self.scanForward()

            if forward < self.obstacleThreshold:
                break

            disturbance, speed = self.algorithm.compute(self.targetForwardDistance, forward, dt)
            if speed < self.minSpeed:
                speed = self.minSpeed
            if speed > self.baseSpeed:
                speed = self.baseSpeed
                
            self.robot.drive(speed, 0)

            travelDist += 0.5 * (prevSpeed + speed) * dt
            prevSpeed   = speed

            self.nextStep()
            self.streamer.send(self.step, disturbance, speed)
            if self.step % self.logInterval == 0:
                print("Step "+str(self.step)+" Forward: "+str(forward)+"mm Speed: "+str(speed)+" mm/s")
                if isinstance(self.algorithm, FuzzyRuleBased):
                    print(self.algorithm.logRules())
            wait(10)

        self.robot.stop()

    def printHeader(self, algorithm):
        self.ev3.speaker.beep()
        print("\nMAZE SOLVER DFS - "+algorithm)
        print("Cell: "+str(self.cellDistanceEW)+"x"+str(self.cellDistanceNS)+"mm")
        print("Open passage threshold: "+str(self.openPassage)+"mm\n")

    def run(self):
        self.printHeader(self.algorithmLabel())
        self.gyro.reset_angle(0)
        self.mazeMap.markVisited()

        try:
            while True:
                openDirs=self.scanJunction()
                print("Cell ("+str(self.mazeMap.x)+","+str(self.mazeMap.y)+")"+" H="+str(self.mazeMap.heading)+" Open="+str(openDirs))
                
                walls=self.mazeMap.storeCellInfo(openDirs)
                if walls is None:
                    walls=self.mazeMap.cellWalls.get((self.mazeMap.x, self.mazeMap.y), 0)
                self.streamer.sendCell(self.mazeMap.x, self.mazeMap.y, walls)
                
                absDir, degreeTurn=self.mazeMap.planMove(openDirs)
                
                if absDir is None:
                    degreeTurn, backDir = self.mazeMap.planBacktrack()

                    if degreeTurn is None:
                        print("Maze fully explored")
                        self.ev3.speaker.beep()
                        break

                    print("Backtrack: turn "+str(degreeTurn)+" deg")

                    self.executeTurn(degreeTurn)
                    wait(200)
                    self.mazeMap.applyTurn(backDir)
                    self.alignHeading()
                    self.centerInCorridor()
                    self.driveCell(backDir)
                    self.mazeMap.commitBacktrack()
                else:
                    print("Move: dir="+str(absDir)+" turn="+str(degreeTurn)+" deg")

                    self.executeTurn(degreeTurn)
                    wait(200)
                    self.mazeMap.applyTurn(absDir)
                    self.alignHeading()
                    self.centerInCorridor()
                    self.driveCell(absDir)
                    self.mazeMap.commitMove()
                
                self.nextStep()
                
        except KeyboardInterrupt:
            self.robot.stop()
            self.ev3.speaker.beep()
            print("\nMAZE SOLVER STOPPED at ("+str(self.mazeMap.x)+","+str(self.mazeMap.y)+")\n")
        finally:
            self.streamer.close()
            
def createController(task, algorithmName):
    configuration=initializeConfiguration(task, algorithmName)
    ev3, robot=hardware_setup(configuration)
    label=task+"-"+algorithmName    
    streamer=WifiStreamer(configuration, label)
    
    if algorithmName=="PID":
        algorithm=PIDAlgorithm(configuration)
    elif algorithmName=="FuzzyPI":
        algorithm=FuzzyPIAlgorithm(configuration)
    elif algorithmName=="FuzzyRuleBased":
        algorithm=FuzzyRuleBased(configuration)
    else:
        raise ValueError("Unknown algorithm: "+algorithmName)

    if task=="LaneKeeping":
        return LaneKeepingController(algorithm, configuration, streamer, ev3, robot)
    elif task=="MazeSolver":
        return MazeSolverController(algorithm, configuration, streamer, ev3, robot)
    else:
        raise ValueError("Unknown task: "+task)
    
def main():
    controller=createController(TASK, ALGORITHM)
    startTime=StopWatch()
    controller.run()
    endTime=startTime.time()
    print("Total time: " + str(endTime) + " ms (" + str(round(endTime / 1000.0, 2)) + " s)")
    
if __name__=="__main__":
    main()