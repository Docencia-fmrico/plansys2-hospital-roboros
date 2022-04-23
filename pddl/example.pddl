(define 
    (problem hospital0)
    (:domain hospital_domain)

(:objects
    robot - robot
    object1 - object

    Room1 Room2 Room3 Room4 Room5 Room6 Room7 Room8 Room9 - waypoint

)

(:init

    (connected Room1 Room2)
    (connected Room2 Room3)
    (connected Room3 Room4)
    (connected Room4 Room5)
    (connected Room5 Room6)
    (connected Room6 Room7)
    (connected Room7 Room8)
    (connected Room8 Room9)

    (connected Room2 Room1)
    (connected Room3 Room2)
    (connected Room4 Room3)
    (connected Room5 Room4)
    (connected Room6 Room5)
    (connected Room7 Room6)
    (connected Room8 Room7)
    (connected Room9 Room8)
    
    (robot_at robot Room7)
    (object_at object1 Room2)
)
    (:goal 
        (and
            (object_at object1 Room8)
            (robot_at robot Room1)
        )
    )
)
