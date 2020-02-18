In this experiment set up, the myoarm is used as a test platform for a BangBang Controller, that can automatically adjust the weights for the different joints to extract the Eigenmode of a systen (e.g. here the arm)

If ou open the TransferFunction Editor in the GUI, you see six TF:
[1] set_record_time : simple function to adjust from which time on, the csv-recorder should record
[2] ReflexController: the Bang Bang controller, implemented after (Lakatos et al, 2013)
[3] set_weight      : TF to change the weighting for the BangBang Controller,
                      e.g. to pick constant weights for a desired certain motion
                           to adjust the weights using the Oja Rule (to excite intrinsic motion)
                           to adjust the weights using the Gradient Descent (to excite intrinsic motion)

In order to start the intrinsic motion, the arm needs to be initially deflected. This can be done manually by using the "Apply Force" Arrow in the NRP GUI, or through the gzclient or through an additional TF.


for further Details on the Controller see:
Lakatos, D., Görner, M., Petit, F., Dietrich, A., & Albu-Schäffer, A. (2013, November). A modally adaptive control for multi-contact cyclic motions in compliantly actuated robotic systems. In 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 5388-5395). IEEE
