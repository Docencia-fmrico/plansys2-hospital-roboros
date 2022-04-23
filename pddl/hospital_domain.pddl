(define (domain hospital)
(:requirements :strips :typing :adl :fluents :durative-actions)

    (:types robot object location door elevator)
    (:predicates
        (robot_at ?r - robot ?l - location)
        (object_at ?o - object ?l - location)
        (opened_door ?d - door)
        (door_joins ?d - door ?l1 - location ?l2 - location)
        (carry_object ?r - robot ?o - object)
        (elevator_joins ?e - elevator ?l1 - location ?l2 - location)
    )

    (:durative-action take_elevator
        :parameters
            (
            ?r - robot
            ?fromloc - location
            ?toloc - location
            ?e - elevator
            )
        :duration 
            (= ?duration 3)
        :condition 
            (and
                (at start (robot_at ?r ?fromloc))
                (at start (elevator_joins ?e ?fromloc ?toloc))
            )
        :effect (and 
            (at start (not(robot_at ?r ?fromloc)))
            (at end (robot_at ?r ?toloc))
        )
    )


    (:durative-action open_door
        :parameters(
            ?r - robot
            ?fromloc - location
            ?toloc - location
            ?d - door
        )
        :duration 
            (= ?duration 3)
        :condition 
            (and
                (at start (robot_at ?r ?fromloc))
                (at start (door_joins ?d ?fromloc ?toloc))
            )
        :effect (and 
            (at end (opened_door ?d))
        )
    )

    (:durative-action move
        :parameters
            (
            ?r - robot
            ?fromloc - location
            ?toloc - location
            ?d - door
            )
        :duration 
            (= ?duration 3)
        :condition 
            (and
                (at start (robot_at ?r ?fromloc))
                (at start (door_joins ?d ?fromloc ?toloc))
                (at start (opened_door ?d))
            )
        :effect (and 
            (at start (not(robot_at ?r ?fromloc)))
            (at end (robot_at ?r ?toloc))
        )
    )

    (:durative-action pick
        :parameters
            (
            ?r - robot
            ?loc - location
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
            ?loc - location
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