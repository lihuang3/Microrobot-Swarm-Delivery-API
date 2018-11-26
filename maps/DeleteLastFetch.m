ii = length(x_obs);
while ~isnan(x_obs(ii))
   x_obs(ii) = [];
   ii = ii - 1;
end

x_obs(ii) = [];

jj = length(y_obs);
while ~isnan(y_obs(jj))
   y_obs(jj) = [];
   jj = jj - 1;
end

y_obs(jj) = [];

