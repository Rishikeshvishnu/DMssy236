
(cl:in-package :asdf)

(defsystem "navigation_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Coord2d" :depends-on ("_package_Coord2d"))
    (:file "_package_Coord2d" :depends-on ("_package"))
  ))