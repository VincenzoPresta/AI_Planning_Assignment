(define (domain task31domain)

  (:requirements :strips :typing :disjunctive-preconditions :durative-actions)

  (:types
    workstation location box content robot content_type 

    carrier slot
    
    workstation box content agent - locable
    box workstation - container
    content box - containable
    robot - agent
    volt current - content_type ; Definizione dei tipi di contenuto
    cart - carrier 
  )
)