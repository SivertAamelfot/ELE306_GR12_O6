% ELE306 Semesteroppgåve Gruppe 12
%
% Dei to første seksjonane er det same som er gjort for hand, berre
% implementert i Matlab v.hj.a Corke Robotics Toolbox
% Programmet er meint å køyre ein og ein seksjon om gongen, bruk
% hurtigtasten (Ctrl + Enter) eller trykk "Run Section".

%%
% 1.i.1) Develop the table of DH-parameters
syms l1 l2 l3 l4 l5 th3
l1 = 0.2; l2 = 0.2; l3 = 0.05; l4 = 0.1; l5 = 0.1;

L1 = Link('d', l1, 'a', 0, 'alpha', pi/2, 'offset', pi/2)
L2 = Link('theta', 3*pi/2, 'a', 0, 'alpha', 3*pi/2, 'offset', l2)
L2.qlim = [0 0.6];
L3 = Link('d', 0, 'a', l3, 'alpha', 3*pi/2, 'offset', 3*pi/2)
L3.qlim = [-pi/2 pi/6]
L4 = Link('theta', 3*pi/2, 'a', 0, 'alpha', 0, 'offset', l4)
L4.qlim = [0 0.4];
L5 = Link('d', l5, 'a', 0, 'alpha', 0);

robotArm = SerialLink([L1, L2, L3, L4, L5], 'name', 'robotArm')
%%
% 1.i.2) Develop the transformation mapping the end-effector to base
% 
% Skaler aksane så det er lettare å sjå
A = [ -0.2 0.7 -0.2 0.7 -0.4 0.3];

% Definerer nødvendige varaibler symbolsk (for å sjå matriser utrekna)
% eller numerisk for å kunne plotte dei
syms l1 l2 l3 l4 l5 t1 t2 t3 t4 te d2 d4
% l1 = 0.2; t1 = 0; 
% l2 = 0.2; d2 = 0; 
% l3 = 0.05; t3 = 0;
% l4 = 0.1; d4 = 0;
% l5 = 0.1; te = 0;
 
% Transformasjonane mellom matrisene. T0 er basen.
T0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

T01 = [-sin(t1) 0 cos(t1) 0; cos(t1) 0 sin(t1) 0; 0 1 0 l1; 0 0 0 1];

T12 = [ 0 0 1 0; -1 0 0 0; 0 -1 0 (l2 + d2); 0 0 0 1];

T23 = [-sin(t3) 0 cos(t3) -sin(t3)*l3; -cos(t3) 0 -sin(t3) -cos(t3)*l3;0 -1 0 0; 0 0 0 1];

T34 = [0 1 0 0; -1 0 0 0; 0 0 1 (l4 + d4); 0 0 0 1];

T4e = [cos(te) -sin(te) 0 0; sin(te) cos(te) 0 0; 0 0 1 l5; 0 0 0 1];

% Reknar ut resultatet av matrisetransformasjon, slik at eg kan plotte dei
pos1 = T0 * T01;
pos2 = pos1 * T12;
pos3 = pos2 * T23;
pos4 = pos3 * T34;
pose = pos4 * T4e

% Teikner alle koordinatsystema og ser at dei stemmer overeins med det eg
% teikna for hand.
% close all
% trplot(T0, 'axis', A, 'length', 0.05);
% hold on
% trplot(pos1, 'length', 0.05)
% trplot(pos2, 'length', 0.05)
% trplot(pos3, 'length', 0.05)
% trplot(pos4, 'length', 0.05)
% trplot(pose, 'length', 0.05)
%%
% 1.ii.1,2 og 3 samt 1.iii.1 er gjort kun for hand.
%%
% 2.i.1) Demonstrate equivalence of forward kinematic solution obtained
% previously by hand
% Køyr 1.i.1 først, slik at robotArm objektet er definert.
q0 = [0 0 0 0 0];

%figure
T = robotArm.fkine(q0)
robotArm.teach(q0)
%%
% 2.i.2) Develop the differential kinematics and demonstrate how it could
% be used.
% Dersom armen står ut til eine sida på båten og båten har ulik fart
% i forhold til bøyene, kan vi kompensere for denne ulikheita som
% vist under. Armen vil då stå i ro i forhold til bøya, sjølv om båten
% køyrer fortare eller saktare enn bøya blir flytta av straumen.
% I eksempelet under køyrer båten raskare enn bøya, og arma må svinge
% bakover for å stå i ro i forhold til bøya.
% Vi ser at qd roterer basen i positiv retning (mot klokka sett ovanfrå) og
% det første prismatiske leddet køyrer bjelken utover.
% Vi ser også av xd at vi kun har fart i negativ x-retning.
% Eg har valt å utelukke siste rad i qd, då end-effector kun kan rotere
% rundt z-aksen, så det er ikkje vits å vise den.
qn = [pi/2 0.5 0 0.2 0]
J = robotArm.jacob0(qn)
qd = pinv(J) * [-0.2 0 0 0 0 0]';
qd(1:4,1)
xd = J*qd;
% J4dof = [J(1:4,:);  J(6,:)]
% qd = inv(J4dof) * [0.2 0 0 0 0]' % sett inn verdi for ønska fart i end-effector
% xd = J4dof * qd;
xd'
robotArm.plot(qn)

%%
% 2.i.3) Develop the inverse kinematics and demonstrate how it could be
% used.
% Dersom vi ønsker at armen skal vere i ein bestemt positur,
% kan vi oppgi denne transformasjonsmatrisa og spøøre kva leddvinklane / 
% lengdene må vere for at vi skal oppnå dette ønsket.
% Sidan vi kun har 5 DOF må vi nytte ei maske, som seier at vi ikkje
% bryr oss om rotasjon rundt eller translasjon langs ein akse.
T = [ 1 0 0 0.5;
      0 0 1 0.5;
      0 -1 0 -0.15
      0 0 0 1]

fram = robotArm.ikine(T, 'mask', [1 1 1 0 1 1])
robotArm.teach(fram)
%%
% 2.i.4 ) Demonstrate example motion planning, on a task relevant to your 
% robot design challenge
% Vi ønsker å rotere armen ut på eine sida av båten for å gjere seg klar
% til å plukke opp ei bøye. Vi har valt å gjere dette i joint-space, sidan
% det er raskare enn cartesian motion, og vi tar betre vare på ledda.
q0 = [0 0 0 0 0];
qn = [pi/2 0.5 0 0.2 0];
startPose = robotArm.fkine(q0)
sluttPose = robotArm.fkine(qn)
t = [0:0.01:2];

punkt = robotArm.jtraj(startPose, sluttPose, t, 'mask', [1 1 1 1 0 1]);
robotArm.plot(punkt)

%%
% 2.ii 1 er diskutert i rapporten i del 3 punkt om kinematisk modell
% 2.ii.2 Vi har bytta ut Bicycle med Unicycle i trajectory control
% (sl_pursuit). Sjå 3.ii.1 nedst i denne fila for simulering.

%%
% 2.iii.1) Demonstrate using the sensory system to command the robot,
% according to the task chosen. That is, show the calculations necessary to
% make the sensory data (e.g. an apple detected at an arbitrary location
% from a static 3D camera) useful to the robot (e.g. calculate the joint
% angles to put the tool point of the end-effector at the apple’s location).

% Skaler aksane så det er lettare å sjå
A = [ -0.5 2 -0.5 1.5 -1.5 1];

% Transformasjon frå kamera til basen av armen
l = 0.8; h3 = 0.2;
TCA = [1 0 0 l;
       0 1 0 0;
       0 0 1 -h3;
       0 0 0 1]
   
% Transformasjon frå kamera til bøya
% Vi har valgt ein vilkårleg vinkel og avstand i alle retninger(unnateke Z),
% men som armen kan nå. Vi tenker at avstanden i Z (høgda) vil vere
% relativt fornuftig, slik at bøya ligg på vassflata og båten ligg passeleg
% djupt i vatnet.
theta = 2.417;
TCB = [cos(theta) sin(theta) 0 0.692;
       sin(theta) -cos(theta) 0 0.43;
       0 0 -1 -0.441;
       0 0 0 1]
       
% Transformasjon frå base av arm til bøye
TAB = inv(TCA) * TCB

close all
figure('Name', 'Bøye og arm i forhold til kamera')
trplot(TCB, 'axis', A, 'color', 'red')
hold on
trplot(TCA, 'color', 'green')

figure('Name', 'Bøye i forhold til arm')
trplot(TAB, 'color', 'red')

% For å nå bøya må leddvinklane / lengdene vere:
qn = [pi/2 0.5 0 0.2 0];
robotArm.ikine(TAB, 'mask', [1 1 1 1 0 1], 'q0', qn)

t = [0:0.01:2];
startPose = robotArm.fkine([pi/2 0.5 0 0 0]);
% Her ønsker vi at armen går til ein positur som er rett over bøya i
% Z-aksen. Setter derfor translasjonsdelen i z-retning i TAB til å vere lik
% translasjonsdelen i z-retning i startPose.
midtPose = TAB;
midtPose(3,4) = startPose.t(3);
sluttPose = TAB;
% Vi kan då sjå vekk frå translasjon i z-retning, bøya kan aldri vere over
% høgda i startposisjonen qn.
punkt1 = robotArm.jtraj(startPose, midtPose, t, 'mask', [1 1 0 1 0 1]);
punkt2 = robotArm.jtraj(midtPose, sluttPose, t, 'mask', [1 1 1 1 0 1]);
robotArm.plot(punkt1, 'fps', 60)
robotArm.plot(punkt2, 'fps', 60)

%%
% 3.i.1) Use motion planning to move the robot end-effector through the
% required positions/orientations for the task chosen
% t = [0:0.01:2]';

startPose = [0 0 0 0 0];
pose1 = [pi/2 0.5 0 0 0];
pose2 = [pi/2 0.5 0 0.4 0];
pose3 = [pi/2 0.2 0 0.4 0];
pose4 = [pi/2 0.2 0 0.1 0];
pose5 = [pi 0.2 0 0.1 0];
pose6 = [pi 0.5 0 0.1 0];
pose7 = [pi 0.5 0 0.2 0];
pose8 = [pi 0.5 0 0.1 0];
pose9 = [pi 0 0 0 0];

punkt1 = mtraj(@lspb, startPose, pose1, 50);
punkt2 = jtraj(pose1, pose2, 50);
punkt3 = jtraj(pose2, pose3, 50);
punkt4 = jtraj(pose3, pose4, 50);
punkt5 = jtraj(pose4, pose5, 50);
punkt6 = jtraj(pose5, pose6, 50);
punkt7 = jtraj(pose6, pose7, 50);
punkt8 = jtraj(pose7, pose8, 50);
punkt9 = jtraj(pose8, pose9, 50);
punkt10 = jtraj(pose9, startPose, 50);

robotArm.plot(punkt1, 'fps', 60)
robotArm.plot(punkt2, 'fps', 60)
robotArm.plot(punkt3, 'fps', 60)
robotArm.plot(punkt4, 'fps', 60)
robotArm.plot(punkt5, 'fps', 60)
robotArm.plot(punkt6, 'fps', 60)
robotArm.plot(punkt7, 'fps', 60)
robotArm.plot(punkt8, 'fps', 60)
robotArm.plot(punkt9, 'fps', 60)
robotArm.plot(punkt10, 'fps', 60)

%%
% 3.i.1) Use motion planning to move the robot end-effector through the
% required positions/orientations for the task chosen
% Arma peiker no akterut, men utfører same bevegelser som over.
t = [0:0.01:1];
q_start = [0 0 0 0 0];
q_venstre = [pi/2 0.5 0 0 0];
q_ned = [pi/2 0.5 0 0.4 0];
q_inn = [pi/2 0.1 0 0.4 0];
q_opp = [pi/2 0.1 0 0 0];
q_hoyre = [0 0.5 0 0 0];
q_ned2 = [0 0.5 0 0.2 0];
q_opp2 = [0 0.5 0 0 0];
q_tilbake = [0 0 0 0 0];

T_start = robotArm.fkine(q_start);
T_venstre = robotArm.fkine(q_venstre);
T_ned = robotArm.fkine(q_ned);
T_inn = robotArm.fkine(q_inn);
T_opp = robotArm.fkine(q_opp);
T_hoyre = robotArm.fkine(q_hoyre);
T_ned2 = robotArm.fkine(q_ned2);
T_opp2 = robotArm.fkine(q_opp2);
T_tilbake = robotArm.fkine(q_tilbake);

punkt1 = robotArm.jtraj(T_start, T_venstre, t, 'mask', [1 1 1 1 0 1]);
punkt2 = robotArm.jtraj(T_venstre, T_ned, t, 'mask', [1 1 1 1 0 1]);
punkt3 = robotArm.jtraj(T_ned, T_inn, t, 'mask', [1 1 1 1 0 1]);
punkt4 = robotArm.jtraj(T_inn, T_opp, t, 'mask', [1 1 1 1 0 1]);
punkt5 = robotArm.jtraj(T_opp, T_hoyre, t, 'mask', [1 1 1 1 0 1]);
punkt6 = robotArm.jtraj(T_hoyre, T_ned2, t, 'mask', [1 1 1 1 0 1]);
punkt7 = robotArm.jtraj(T_ned2, T_opp2, t, 'mask', [1 1 1 1 0 1]);
punkt8 = robotArm.jtraj(T_opp2, T_tilbake, t, 'mask', [1 1 1 1 0 1]);

robotArm.plot(punkt1, 'fps', 60)
robotArm.plot(punkt2, 'fps', 60)
robotArm.plot(punkt3, 'fps', 60)
robotArm.plot(punkt4, 'fps', 60)
robotArm.plot(punkt5, 'fps', 60)
robotArm.plot(punkt6, 'fps', 60)
robotArm.plot(punkt7, 'fps', 60)
robotArm.plot(punkt8, 'fps', 60)

%%
% 3.ii.1) Simulate your chosen challenge, and discuss the simulation 
% results in terms of chosen control strategy and performance
% Vi utfører kun simulering v.hj.a. Simulink her. Sjå rapporten for
% diskjon. Vi antar at bøya reiser i ei rett linje med straumen.
sl_pursuit_unicycle;