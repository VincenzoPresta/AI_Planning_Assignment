(define (domain task31domain)

  (:requirements :strips :typing :durative-actions)

  (:types
    workstation location box content robot contentType 

    carrier slot
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt bolt - contenType ; Definizione dei tipi di contenuto
    cart - carrier              ; carrier is generic, there can be more carrier type
  )

    (:predicates
        (at ?objectLoc - locable ?location - location )
        (contain ?container - container ?containable - containable)
        (box-is-empty ?box - box)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?type - contentType) ; Indica di quale tipo di contenuto ha bisogno una workstation
        (is-type ?content - content ?contentType - contentType); Associa contenuti specifici ai loro tipi

        ;modifiche per istanza no. 2
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier
        (carrier-has-slot ?carrier - carrier ?slot - slot)
        (slot-has-box ?slot - slot ?box - box)
        (free ?slot - slot)
    )

    ;in questo dominio si suppone che la empty to workstation sia possibile solo se la scatola si trova nella workstation
    ;è possibile effettuare fill e pickup in parallelo

    (:durative-action fill-box-from-location
        :parameters (?agent - agent ?box - box ?content - content ?loc - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (box-is-empty ?box))
            (at start (at ?content ?loc))
            (over all (at ?agent ?loc))
            (over all (at ?box ?loc))
        )
        :effect (and 
            (at start (not (box-is-empty ?box)))
            (at start (not (at ?content ?loc)))
            (at end (contain ?box ?content))
        )
    )

    (:durative-action fill-box-from-workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - contentType ?loc - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (box-is-empty ?box))
            (at start (contain ?workstation ?content))
            (at start (workstation-has-type ?workstation ?type))
            (over all (at ?agent ?loc))
            (over all (at ?workstation ?loc))
            (over all (contain ?workstation ?box))
            (over all (is-type ?content ?type))
        )
        :effect (and 
            (at start (not (box-is-empty ?box)))
            (at start (not (contain ?workstation ?content)))
            (at end (not (workstation-has-type ?workstation ?type)))
            (at end (contain ?box ?content))
        )
    )

    (:durative-action empty-box-workstation
        :parameters (?agent - agent ?box - box ?content - content ?type - contentType ?workstation - workstation ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (contain ?box ?content))
            (over all (at ?agent ?location))
            (over all (is-type ?content ?type))
            (over all (at ?workstation ?location))
            (over all (contain ?workstation ?box))
        )
        :effect (and 
            (at start (not (contain ?box ?content)))
            (at end (workstation-has-type ?workstation ?type))
            (at end (box-is-empty ?box))
            (at end (contain ?workstation ?content))
        )
    )

    (:durative-action empty-box-location
        :parameters (?agent - agent ?box - box ?content - content ?location - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (contain ?box ?content))
            (over all (at ?agent ?location))
            (over all (at ?box ?location))
        )
        :effect (and 
            (at start (not (contain ?box ?content)))
            (at end (at ?content ?location))
            (at end (box-is-empty ?box))
        )
    )

    (:durative-action pick-up-from-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location )
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?slot))
            (at start (at ?box ?location))

            (over all (at ?agent ?location))
            (over all (agent-has-carrier ?agent ?carrier))
            (over all (carrier-has-slot ?carrier ?slot))
        )
        :effect (and 
            (at start (not (at ?box ?location)))
            (at start (not (free ?slot)))
            (at end (slot-has-box ?slot ?box))
        )
    )

    (:durative-action pick-up-from-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (free ?slot))
            (at start (contain ?workstation ?box))
            (over all (at ?workstation ?location))
            (over all (at ?agent ?location))
            (over all (agent-has-carrier ?agent ?carrier))
            (over all (carrier-has-slot ?carrier ?slot))
        )
        :effect (and 
            (at start (not (contain ?workstation ?box)))
            (at start (not (free ?slot)))
            (at end (slot-has-box ?slot ?box))
        )
    )
    

    (:durative-action move
        :parameters (?agent - agent ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and 
            (at start (at ?agent ?from))
            (over all (connected ?from ?to))
        )
        :effect (and 
            (at start (not (at ?agent ?from)))
            (at end (at ?agent ?to))
        )
    )


    (:durative-action deliver-to-workstation
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?workstation - workstation ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (slot-has-box ?slot ?box))
            (over all (carrier-has-slot ?carrier ?slot))
            (over all (agent-has-carrier ?agent ?carrier))
            (over all (at ?agent ?location))
            (over all (at ?workstation ?location))
        )
        :effect (and 
            (at start (not (slot-has-box ?slot ?box)))
            (at end (free ?slot))
            (at end (contain ?workstation ?box))
        )
    )

    (:durative-action deliver-to-location
        :parameters (?agent - agent ?carrier - carrier ?slot - slot ?box - box ?location - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (slot-has-box ?slot ?box))
            (over all (at ?agent ?location))
            (over all (carrier-has-slot ?carrier ?slot))
            (over all (agent-has-carrier ?agent ?carrier))
        )
        :effect (and 
            (at start (not (slot-has-box ?slot ?box)))
            (at end (free ?slot))
            (at end (at ?box ?location))
        )
    )
)