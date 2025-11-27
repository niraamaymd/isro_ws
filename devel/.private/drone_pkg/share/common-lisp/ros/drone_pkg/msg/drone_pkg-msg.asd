
(cl:in-package :asdf)

(defsystem "drone_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GridDetection" :depends-on ("_package_GridDetection"))
    (:file "_package_GridDetection" :depends-on ("_package"))
  ))