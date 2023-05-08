# StartQuad

## StartRealQuad (Service)

Service: "/combo/start_real_quad"

```
string id
---
int64 status
```

## StartSimQuad (Service)

Service: "/combo/start_sim_quad"

```
string id
---
int64 status
```

# Move

## MoveVel

Service: "/combo/{id}/move_vel"

- Neue Daten schicken sobald berechnet
- Stoppen: 0 Speed und 0-Vektor

```
geometry_msgs/Twist velocity
---
int64 status
```

## MovePos

Service: "/combo/{id}/move_pos"

```
geometry_msgs/Pose pose
---
int64 status
```

- geometry_msgs.POSE

- Nicht hochfrequent (falls weniger als 20hz; andernfalls alle ca 2s)
- Low-level flyTo & HighLevel flyTo

- Neue Daten schicken sobald berechnet
- Orientierung erstmal 0-Vektor

# GetQuads

Service: "/combo/get_quads"

```
---
mgs/IdObject[] objects
string error
```

`msg/IdObject.mgs`

```
string id
bool isSim
```

- return alle ids für quads => ["cf3", "cf5"]

# GetPositions

Topic: "/combo/{id}/position"

```
---
geometry_msgs/Pose
```

# GetQuadStates

Topic: "/combo/{id}/quad_state"

```
{
    "battery": 30,
    "quad_staate": "EXECUTING" # "STOPPING", "LOADING", "STARTING", "IDLE"
}
```

- battery
- battery_status

# PosReached

Topic: "/combo/{id}/pos_reached"
bool posReached

Sollte einfach einen bool in das topic schmeißen, wenn moveTo erreicht wurde (Aktuelle Prio ist gering)

# Environment

Service: "/combo/environment"
---
Metadaten zu environment
Bonding Box: 8 Punkte des Quaders aus dem die Quads nicht rausdürfen
```geometry_msgs.Polygon```
Zu klären: Was wenn nur simulierte?

# Zauberstab

Service: "/combo/start_wand"

```
string id
---
int64 status
```

=> Now we should be able to use move_pos and move_vel with the given id (spawn services like for quad except state)

- `/combo/{wand-123}/move_to` => no interpolation, directly set pos
- `/combo/{wand-123}/move_vel` => move vel like quad
- `/combo/{wand-123}/position` (service and topic)

---
Service: "/combo/get_wands"

```
---
mgs/IdObject[] objects
int64 status
```

# Hint

service and topic names implemented in `ros1java/src/main/java/combo/utils/ServiceAndTopicNames.java`