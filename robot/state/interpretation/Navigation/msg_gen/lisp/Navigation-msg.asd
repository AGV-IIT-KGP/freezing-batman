
(cl:in-package :asdf)

(defsystem "Navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RazorImu" :depends-on ("_package_RazorImu"))
    (:file "_package_RazorImu" :depends-on ("_package"))
  ))