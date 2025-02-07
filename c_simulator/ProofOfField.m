%display vector fields


%sink at 1.1,1.1 source and -1.1,-1.1

resolution = 20;

X = zeros(20^2, 1);
Y=X;
U=X;
V=X;

%evalutation points
source_pos = [-1.1, -1.1];
sink_pos = [1.1, 1.1];

x_vals = linspace(-3,3,resolution);
y_vals = linspace(-3,3,resolution);

x_count = 1;
y_count = 1;
while(x_count < resolution + 1)
    y_count = 1;
    while(y_count < resolution + 1)

        r_source = norm([x_vals(x_count), y_vals(y_count)] - source_pos);
        r_sink = norm([x_vals(x_count), y_vals(y_count)] - sink_pos);

        u_source = ([x_vals(x_count), y_vals(y_count)] - source_pos) / r_source;
        u_sink = ([x_vals(x_count), y_vals(y_count)] - source_pos) / r_sink;

        v = -1/(r_source^2) * u_source + 1/(r_sink^2) * u_sink;

        X(y_count + resolution * (x_count - 1)) = x_vals(x_count);
        Y(y_count + resolution * (x_count - 1)) = y_vals(y_count);
        U(y_count + resolution * (x_count - 1)) = v(1);
        V(y_count + resolution * (x_count - 1)) = v(2);


        y_count = y_count + 1;
    end
    x_count = x_count + 1;
end

quiver(X,Y,U,V)
