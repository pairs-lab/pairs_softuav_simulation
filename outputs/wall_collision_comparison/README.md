# Qualitative rigid impact progression

TRANSIENT PROFILE: after contact, assumed observation delay increases from 12 ms to 50 ms for 2.8 s, then returns to 12 ms. The same parameters apply at every speed. This is a deliberately selected illustrative control-response scenario, NOT a measured IMU effect, matched-controller comparison, or proof that compliance alone explains the difference. No pose/velocity kicks or preset lean are applied. The reference video (1:50–approximately 3:12) shows hover arm disturbances, NOT known-speed wall impacts. It does not establish a 2–5 m/s response curve.

Includes 2–7 m/s every .5 m/s and 7.6 m/s; omits 1.5,7.5,8 m/s.
Soft is the approved compression-then-controlled-bank profile, unchanged. Rigid has physical guards and starts level, aligned with travel. No heading offset, forced torque or asymmetric contact setup is added. A symmetric frontal collision need not produce substantial yaw. The same rigid controller gains/motor response/material assumptions are used at every speed. No speed-branch scripts force instability or fracture.
Rigid observations are sampled at nominal 150 Hz with 12 ms delay. A solved wall shock above an assumed 16 g holds observations for an assumed 40 ms; this can produce yaw/tilt overshoot naturally. These are sensitivity assumptions, NOT Skydio X2 measurements. Arm fracture retains the original 110 Nm /2000 N assumed contact-load thresholds. Rotors stop only when their arm physically detaches via weld release. No propeller damage rule.
The 2–3 /3–5 /5–7.6 m/s descriptions are qualitative expectations, not calibrated probabilities or guarantees. Inspect JSON for measured impact speed, body deceleration, first-impact angular rate, yaw, tilt and fracture.
This is an illustrative comparison, NOT a controlled hardware superiority test: soft is frontal with an added bank reference; rigid is frontal with finite feedback. Insets/overlay now show yaw deviation as well as tilt.
Previous videos and narrow-passage control are unchanged.
Context: https://arxiv.org/html/2105.12273v1 , https://arxiv.org/html/2605.25790v1 .
