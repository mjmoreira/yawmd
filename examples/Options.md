```
# ifaces.ids is required. The other settings are optional.
ifaces: 
{
  ids = ["<mac address 1>", "<mac address 2>", "<mac address 3>"];
  enable_interference = <true | false>;
  # indices correspond to the ifaces.ids position
  links = ((<source index>, <destination index>, <snr>), ...);
};

# All the following are optional, but some are required once the
# setting commented above is present.
model:
{
  noise_threshold = <int>;
  fading_coefficient = <int>;
  type = < "snr" | "prob" | "path_loss" >;
  # .type = "path_loss"
  name = < "free_space" | "log_distance" | "log_normal_shadowing" |
           "two_ray_ground" | "itu" >;
  # Optional: used by .type = < "snr" | "prob" >
  links = ((<source>, <destination>, <snr value | prob value>), ...);
  # Optional: used by .type = "prob"
  default_prob = <float>;
  # Optional: used by .type = "snr"
  default_snr = <int>;
  # .type = "path_loss"
  positions = ((<float_x>, <float_y>, <float_z>), ...);
  # .type = "path_loss"
  tx_powers = [<float>, <float>, ...];
  # Optional: used by .type = "path_loss"
  directions = ((<float_x>, <float_y>, <float_z>), ...);
  # Optional: used by .type = "path_loss"
  isnodeaps = [<int>, <int>, ...];
  ## The following are required if type = "path_loss" and if name=x
  # .name = < "log_distance" | "log_normal_shadowing" >
  path_loss_exp = <float>;
  # .name = "log_distance"
  xg = <float>;
  # .name = <"free_space"| "log_normal_shadowing"| "two_ray_ground">
  sL = <int>;
  # .name = "itu"
  nFLOORS = <int>; lF = <int>; pL = <int>;
}; 
```