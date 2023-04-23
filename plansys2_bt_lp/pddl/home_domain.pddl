;; Copyright 2023 Looping
;;
;; Licensed under the Apache License, Version 2.0 (the "License");
;; you may not use this file except in compliance with the License.
;; You may obtain a copy of the License at
;;
;;     http://www.apache.org/licenses/LICENSE-2.0
;;
;; Unless required by applicable law or agreed to in writing, software
;; distributed under the License is distributed on an "AS IS" BASIS,
;; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
;; See the License for the specific language governing permissions and
;; limitations under the License.

(define (domain home-move)
(:requirements :strips :typing :equality :durative-actions :negative-preconditions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
  room corridor - location
  door
  robot
  item
  grandma
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates 
  (robot_at ?r - robot ?l - location)
  (grandma_at ?g - grandma ?l - location)
  (item_at ?o - item ?l - location)
  (robot_carrying ?r - robot ?o - item)
  (robot_free ?r - robot)
  (connected_door ?l1 ?l2 - location ?d - door)
  (connected ?l1 ?l2 - location)
  (open ?d - door)
  (close ?d - door)

  (grandma_wants ?i - item)
  (grandma_has_item)
  (grandma_req_open_door ?g - grandma ?d - door)
  (grandma_get_open_door)
  (item_should_be ?i - item ?l - location)
  (item_organized ?i - item)
  (item_disorganized ?i - item)
  (no_grandma_chores)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action grandma_chores
  :parameters (?r - robot ?g - grandma)
  :duration (= ?duration 1)
  :condition (and
      (at end (and
        ;grandma has all items that request
        (grandma_has_item)
        (grandma_get_open_door)
      ))
  )
  :effect (and
      (at end(no_grandma_chores))
  )
)

(:durative-action item_for_grandma
  :parameters (?r - robot ?from ?to - location ?i - item ?g - grandma)
  :duration (= ?duration 20)
  :condition (and
      (at start(and
          (grandma_wants ?i)
          (robot_free ?r)
          (grandma_at ?g ?to)
          (item_at ?i ?from)
      ))
      (at end(and
          (item_at ?i ?to)
          (robot_at ?r ?to)
          (robot_free ?r)
      ))
  )
  :effect 
      (and
        (at end(not(grandma_wants ?i)))
        (at end(grandma_has_item))
      )
)

(:durative-action organize_item
  :parameters (?to - location ?i - item)
  :duration (= ?duration 10)
  :condition 
        (and
          (at start (item_disorganized ?i))
          (over all (and
            (no_grandma_chores)
            (item_should_be ?i ?to)
          ))
          (at end (item_at ?i ?to))
        )
  :effect 
      (and
        (at end(not(item_disorganized ?i)))
        (at end(item_organized ?i))
      )
)

(:durative-action open_door
  :parameters (?r - robot ?l1 ?l2 - location ?d - door)
  :duration(= ?duration 3)
  :condition 
    (and 
      (at start(close ?d))
      (over all(robot_at ?r ?l1))
      (over all(connected_door ?l1 ?l2 ?d))
    )
  :effect 
    (and 
      (at start(not (close ?d)))
      (at end(open ?d))
    )
)

(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration(= ?duration 6)
  :condition 
    (and
      (at start(robot_at ?r ?from))
      (over all(connected ?from ?to)) 
    )
  :effect 
    (and
      (at start(not(robot_at ?r ?from)))
      (at end(robot_at ?r ?to))
    )
)

(:durative-action move_through_door
  :parameters (?r - robot ?from ?to - location ?d - door)
  :duration(= ?duration 6)
  :condition 
    (and 
      (at start(robot_at ?r ?from))
      (over all(open ?d))
      (over all(connected_door ?from ?to ?d))
    ) 
  :effect 
    (and
      (at start(not(robot_at ?r ?from)))
      (at end(robot_at ?r ?to))
    )
)

(:durative-action drop_item
  :parameters (?r - robot ?o - item ?l - location)
  :duration(= ?duration 1)
  :condition 
    (and 
      (at start(robot_carrying ?r ?o))
      (over all(robot_at ?r ?l))
    )
  :effect
    (and 
      (at start(not(robot_carrying ?r ?o)))
      (at end(robot_free ?r))
      (at end(item_at ?o ?l))
    )
)


(:durative-action pick_item
  :parameters (?r - robot ?i - item ?l - location)
  :duration(= ?duration 1)
  :condition 
    (and 
      (at start(item_at ?i ?l))
      (at start(robot_free ?r))
      (over all(robot_at ?r ?l))
    )
  :effect 
    (and
      (at start(not(item_at ?i ?l)))
      (at start(not(robot_free ?r)))
      (at end(robot_carrying ?r ?i))
    )
)

(:durative-action request_open_door
  :parameters (?r - robot ?d - door ?g - grandma)
  :duration (= ?duration 16)
  :condition (and
      (at start(close ?d))
      (at start(grandma_req_open_door ?g ?d))
      (at end(open ?d))
  )
  :effect (and
      (at end(and
      (not(grandma_req_open_door ?g ?d))
      (grandma_get_open_door)
      ))
  )
);; end Actions ;;;;;;;;;;;;;;;;;;;;;;;;

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;