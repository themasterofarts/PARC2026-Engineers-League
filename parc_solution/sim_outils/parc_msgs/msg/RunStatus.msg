builtin_interfaces/Time stamp
uint8 IDLE       = 0       # aucune course en cours
uint8 ARMED      = 1       # ordre envoye, attente du depart
uint8 RUNNING    = 2       # chronometre en marche
uint8 FINISHED   = 3       # dernier tag, finish ou juge
uint8 TIMED_OUT  = 4       # 10 minutes atteintes
uint8 TERMINATED = 5       # arret de securite par un juge
uint8 state
float32 elapsed_s
float32 time_limit_s       # 600
uint8[] tag_order
uint8[] tags_reported      # recues, pas encore jugees
uint8 start_method         # StartSignal.METHOD_*
bool false_start           # mouvement avant le depart
