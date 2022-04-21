(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
    robot
    room
)

(:predicates
(robot_at ?r - robot ?ro - room)
(connected ?ro1 ?ro2 - room)
)

(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?r1 ?r2))
        (at start(robot_at ?r ?r1))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

)