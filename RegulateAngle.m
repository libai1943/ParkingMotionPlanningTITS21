function angle  = RegulateAngle(angle)
while (angle > 2 * pi)
    angle = angle - 2 * pi;
end
while (angle < 0)
    angle = angle + 2 * pi;
end
end