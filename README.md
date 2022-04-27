# Motion retargeting solver for target human character
* This project is based on Levmar(Levenberg-Marquardt algorithm) to create retargeting results for target human character.
* Our framework is simple. you just specify the path of source BVH or FBX files, and then you can get the retargeted BVH files.
* This project is used for research purpose only, and we hope that our work helps you to get human-motion data for deep-learning. 

## Disclaimer
This is a on-going work, so 
* We haven't seen a harmful error yet, it will occur implementation issues such as memory leak.
* It might need a post-processing such as foot-contact preserving (avoid foot-skating).
* We implemented our work based on our lab fundamental framework called MotionWorks. Please check license file.<br>(KAIST, Motion Computing Lab:https://motionlab.kaist.ac.kr/)<br><br> 

We tested the solver to mocap data,
  * Opti-track(https://optitrack.com/) 
  * AxisNeuron(https://neuronmocap.com/content/axis-neuron) 
  * KinectV2 mocap using IPIsoft(http://ipisoft.com/) 
  * Mixamo(https://www.mixamo.com) 
  * CMU(http://mocap.cs.cmu.edu/)

## Target Character
The target character is similar with Mixamo (https://www.mixamo.com) Character, we remove the finger joints.
You can see the character information on the "mixamo_rest.txt" file

## Retargeting Procedure [using Motion-Works Framework]
1. First, executing the bat file to download all library files, it will automatically download in the Libraries folder.
![image](https://user-images.githubusercontent.com/41664042/107373757-36f8b700-6b2a-11eb-9ccb-f903f3ff4206.png)
2. copy and paste all files in the Libraries folder to main Libraries folder 
![image](https://user-images.githubusercontent.com/41664042/107373807-45df6980-6b2a-11eb-98ac-28bf36d06907.png)
3. please run the Examples/viewRetargeting project
![image](https://user-images.githubusercontent.com/41664042/107373897-5d1e5700-6b2a-11eb-8669-aad9df8a4061.png)
4. ..further guidelines are to be announced.
<br>if you have some issues, please let me know using issues page.

## Retargeting Procedure [using only executive file]
1. Please run the exe file in the Release folder

2. Specify your source directory
![화면 캡처 2021-02-08 234230](https://user-images.githubusercontent.com/41664042/107235252-d99c3180-6a67-11eb-9fb8-596c1e78e308.png)

3. Specify your retarget directory
![화면 캡처 2021-02-08 234328](https://user-images.githubusercontent.com/41664042/107235278-e28d0300-6a67-11eb-9cb4-a62d5533e52f.png)

4. Select enum index to specify your retargeting process 
![Untitled (2)](https://user-images.githubusercontent.com/41664042/107233452-f8012d80-6a65-11eb-89db-45a64cc4b080.png)<br>
(if you want to retarget your own character, you should modify the code [to be announced])<br>

5. Sometimes, it will fail to load. we should modify our character text file (it will be in the retarget folder).
  ![Untitled (3)](https://user-images.githubusercontent.com/41664042/107233655-267f0880-6a66-11eb-9de0-0a3829c650bc.png)<br>
  We assume that character has a single "Free Joint" placed on hip and all others have a "Ball Joint".

5. That's it! we automatically gather all bvh files inside a source folder, and retarget all!
![Untitled (4)](https://user-images.githubusercontent.com/41664042/107233876-634aff80-6a66-11eb-942f-f97df9dbfbc0.png)

## System Requirements
* The project is developed under Windows 10, using Visual Studio 2015 x64, C++ and have never been tested in other environments.

## Dependencies (we plan to publish our main framework, it is to be announced.)
* OpenSceneGraph354
* blas
* lapack
* BulletEngine
* Levmar(http://users.ics.forth.gr/~lourakis/levmar/)
