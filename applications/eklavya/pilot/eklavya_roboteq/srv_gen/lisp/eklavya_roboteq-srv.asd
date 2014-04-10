
(cl:in-package :asdf)

(defsystem "eklavya_roboteq-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetSpeed" :depends-on ("_package_SetSpeed"))
    (:file "_package_SetSpeed" :depends-on ("_package"))
    (:file "GetSpeed" :depends-on ("_package_GetSpeed"))
    (:file "_package_GetSpeed" :depends-on ("_package"))
  ))