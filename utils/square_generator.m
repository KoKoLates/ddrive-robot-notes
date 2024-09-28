function waypoints = square_generator(x, y, border)
    step = 5;
    waypoints = [];

    waypoints = [waypoints; (x:step:x + border)', repmat(y, border/step + 1, 1)];
    waypoints = [waypoints; repmat(x + border, border/step + 1, 1), (y:step:y + border)'];
    waypoints = [waypoints; (x + border:-step:x)', repmat(y + border, border/step + 1, 1)];
    waypoints = [waypoints; repmat(x, border/step + 1, 1), (y + border:-step:y)'];
end
