
(cl:in-package :asdf)

(defsystem "global_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Seed" :depends-on ("_package_Seed"))
    (:file "_package_Seed" :depends-on ("_package"))
  ))