(define (domain hospital_domain)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
    robot
    waypoint
    object
    )
(:predicates

    (robot_at ?r - robot ?wp - waypoint)
    (connected ?wp1 ?wp2 - waypoint)
    (object_at ?o - object ?l - waypoint)
    (carry_object ?r - robot ?o - object)

)

(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action pick
    :parameters
        (
        ?r - robot
        ?loc - waypoint
        ?o - object
        )
    :duration 
        (= ?duration 3)
    :condition 
        (and
            (at start (robot_at ?r ?loc))
            (at start (object_at ?o ?loc))
        )
    :effect (and 
        (at end (carry_object ?r ?o))
        (at start (not(object_at ?o ?loc)))
    )
)

(:durative-action drop
    :parameters
        (
        ?r - robot
        ?loc - waypoint
        ?o - object
        )
    :duration 
        (= ?duration 3)
    :condition 
        (and
            (at start (carry_object ?r ?o))
            (at start (robot_at ?r ?loc))
        )
    :effect (and 
        (at start (not(carry_object ?r ?o)))
        (at end (object_at ?o ?loc))
    )
)

)
