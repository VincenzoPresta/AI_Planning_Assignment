(define (domain task1domain)

    (:requirements :strips :typing :disjunctive-preconditions :numeric-fluents)

    (:types
    workstation location box content robot content_type carrier
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt current - content_type ; Definizione dei tipi di contenuto
    cart - carrier              ; carrier is generic, there can be more carrier type
    )

    (:predicates
        (at ?objectLoc - locable ?location - location )
        (contain ?container - container ?containable - containable)
        (box-is-empty ?box - box)
        ;(free ?agent - agent)
        (connected ?loc1 - location ?loc2 - location)
        (workstation-has-type ?workstation - workstation ?type - content_type) ; Indica di quale tipo di contenuto ha bisogno una workstation
        (is-type ?content - content ?content_type - content_type); Associa contenuti specifici ai loro tipi

        ;modifiche per istanza no. 2
        (carrier-has-box ?carrier - carrier ?box - box)                   ;true se carrier ha box
        (agent-has-carrier ?agent - agent ?carrier - carrier)             ;true se agent ha il carrier
    )



    ; 1: ogni agente ha un "carrier" con una capacità massima (può essere diversa per ogni agente)       ;DONE
    ; 2: l'agente può caricare le scatole nel "carrier" fino alla capacità massima
    ; 3: l'agente e le scatole da caricare devono essere nella stessa location
    ; 4: l'agente può muovere il carrello ad una locazione dove le ws hanno bisogno dei materiali
    ; 5: l'agente può scaricare una o più scatole dal "carrier" alla locazione dove si trova
    ; 6: l'agente può continuare a muoversi verso un'altra locazione, scaricare altre scatole senza tornare al warehouse ogni volta
    ; 7: diversi tipi di "carrier" 
    ; 8: per ogni agente bisogna contare e tenere traccia di: i) quali scatole sono sul suo "carrier" ii) quante scatole ci sono sul carrier (per non superare la capacità massima) ;DONE
    ; 9: la capacità del "carrier" deve essere "problem specific", quindi deve essere definita nel file del problema.                                                               ;DONE



    (:functions
        (curr-carrier-load ?carrier - carrier) ; funzione che restituisce un intero, rappresentante il carico corrente del carrier
        (capacity ?carrier - carrier ) ;funzione che restituisce un intero, rappresentante il carico corrente del carrier
    )

    ;l'assunzione fatta per l'istanza 1 : robot caricato con una scatola ha le "mani" occupate quindi non può eseguire empty e fill, cade in questo caso.
    ;si suppone, visto l'utilizzo di un "carrier", che il robot può eseguire le operazioni di empty e fill
    ;first try: eliminato il predicato free

    (:action fill_box_from_location
        :parameters (?agent - agent ?box - box ?content - content ?loc - location )
        :precondition (and 
                        (at ?agent ?loc)
                        (at ?box ?loc)
                        (at ?content ?loc)

                        ;(free ?agent)
                        (box-is-empty ?box)
        )      
        :effect (and 
                    (not (box-is-empty ?box))
                    (not (at ?content ?loc))
                    (contain ?box ?content)
        )
    )

    (:action fill_box_from_workstation
        :parameters (?agent - agent ?box - box ?content - content ?workstation - workstation ?type - content_type ?loc - location)
        :precondition (and 
                        (at ?agent ?loc)
                        (at ?workstation ?loc)
                        (contain ?workstation ?box)
                        (contain ?workstation ?content)
                        (is-type ?content ?type)
                        (workstation-has-type ?workstation ?type)

                        ;(free ?agent)
                        (box-is-empty ?box)
        )                   
        :effect (and 
                (not (box-is-empty ?box))
                (not (contain ?workstation ?content))
                (not (workstation-has-type ?workstation ?type))
                (contain ?box ?content)
        )
    )

    ; La box può essere svuotata sia se è nella workstation sia se è nella location
    (:action empty-box-ws
        :parameters (?agent - agent ?box - box ?content - content ?content_type - content_type ?workstation - workstation ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (or (at ?box ?loc) (contain ?workstation ?box))
                            (at ?workstation ?loc)

                            ;(free ?agent)
                            (contain ?box ?content)
                            (is-type ?content ?content_type)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (box-is-empty ?box)
                    (contain ?workstation ?content)
                    (workstation-has-type ?workstation ?content_type)
        )
    )

    (:action empty-box-loc
        :parameters (?agent - agent ?box - box ?content - content ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?box ?loc) 
        
                            ;(free ?agent)
                            (contain ?box ?content)
        )
        :effect (and 
                    (not (contain ?box ?content))
                    (box-is-empty ?box)

                    (at ?content ?loc)
        )
    )

    (:action pick-up-from-ws
        :parameters (?agent - agent ?carrier - carrier ?box - box ?workstation - workstation ?loc - location)
        :precondition (and 
                    (at ?agent ?loc)
                    (at ?workstation ?loc)
                    (contain ?workstation ?box)
                    (agent-has-carrier ?agent ?carrier)
                    (< (curr-carrier-load ?carrier) (capacity ?carrier)) ; verifica che ci sia spazio nel carrier, in particolare la precondizione fallisce se il carrier è già pieno, senza bisogno di un predicato booleano separato. 
        )
        :effect (and 
            (not (contain ?workstation ?box))
            (carrier-has-box ?carrier ?box)
            (increase (curr-carrier-load ?carrier) 1)  ; aumenta il carico corrente del carrier di 1
        )
    )

    (:action pick-up-from-location
        :parameters (?agent - agent ?carrier - carrier  ?box - box ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?box ?loc)
                            (agent-has-carrier ?agent ?carrier)
                            (< (curr-carrier-load ?carrier) (capacity ?carrier))

        )
        :effect (and 
                    (not (at ?box ?loc))
                    (carrier-has-box ?carrier ?box)
                    (increase (curr-carrier-load ?carrier) 1)  ; aumenta il carico corrente del carrier di 1
        )
    )

    (:action move
        :parameters (?agent - agent ?loc1 - location ?loc2 - location)
        :precondition (and 
                        (or (connected ?loc1 ?loc2) (connected ?loc2 ?loc1))
                        (at ?agent ?loc1)
        )
        :effect (and 
                    (not (at ?agent ?loc1))
                    (at ?agent ?loc2)
        )
    )

    (:action deliver-to-ws
        :parameters (?agent - agent ?carrier - carrier ?box - box ?workstation - workstation ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (at ?workstation ?loc)
                            
                            (agent-has-carrier ?agent ?carrier)
                            (carrier-has-box ?carrier ?box)
        )
        :effect (and 
                    (not (carrier-has-box ?carrier ?box))
                    (contain ?workstation ?box)
                    (decrease (curr-carrier-load ?carrier) 1)
        )
    )

    (:action deliver-to-loc
        :parameters (?agent - agent ?carrier - carrier ?box - box ?loc - location)
        :precondition (and 
                            (at ?agent ?loc)
                            (carrier-has-box ?carrier ?box)
        )
        :effect (and 
                    (not (carrier-has-box ?carrier ?box))
                    (at ?box ?loc)
                    (decrease (curr-carrier-load ?carrier) 1)
        )
    )

)

