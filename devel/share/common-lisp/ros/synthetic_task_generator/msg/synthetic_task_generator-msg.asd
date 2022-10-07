
(cl:in-package :asdf)

(defsystem "synthetic_task_generator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SyntheticTaskMsg" :depends-on ("_package_SyntheticTaskMsg"))
    (:file "_package_SyntheticTaskMsg" :depends-on ("_package"))
  ))