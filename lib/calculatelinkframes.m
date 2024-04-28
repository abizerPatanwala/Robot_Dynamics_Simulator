function [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot)
  Mj1 =  robot.links(1).A(0).T;
  Mj2 = Mj1 * robot.links(2).A(0).T;
  Mj3 = Mj2 * robot.links(3).A(0).T;
  Mj4 = Mj3 * robot.links(4).A(0).T;
  Mj5 = Mj4 * robot.links(5).A(0).T;
  Mj6 = Mj5 * robot.links(6).A(0).T;
  
  M1 = Mj1 * [eye(3) robot.links(1).r'; 0 0 0 1];
  M2 = Mj2 * [eye(3) robot.links(2).r'; 0 0 0 1];
  M3 = Mj3 * [eye(3) robot.links(3).r'; 0 0 0 1];
  M4 = Mj4 * [eye(3) robot.links(4).r'; 0 0 0 1];
  M5 = Mj5 * [eye(3) robot.links(5).r'; 0 0 0 1];
  M6 = Mj6 * [eye(3) robot.links(6).r'; 0 0 0 1];
  
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = eye(4);
end

