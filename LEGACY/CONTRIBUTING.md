Contributing to RoboBuggy
=========================

Welcome! We're glad you've decided to help build CMU's first autonomous buggy. In order to keep the buggy running well, we have a few light standards for editing code in the repository.

## Getting an Assignment

Todo

## Formulating a Change

We encourage using branches so that you can work on a change, make it visible to the group for review and feedback, and merge once the idea is fully baked. This makes it easy for everyone to work on different tasks without having to work about impacting others or the stability of master.

You can make a branch either on your local clone (and push later) or on the github interface (and pull the new branch now). Change to the branch with `git checkout -b <branch_name>`. Please make you branch names with the `<username>_<quick_change_description>` format so that we can find them easily.

Once you have tested/debugged your change and want it to be considered for merge into master, use it to submit a pull request.

## Pull Requests (Review)

The master branch of RoboBuggy is protected such that you cannot force push to it or push commits directly to master (to prevent [1]). Instead, changes must come from a pull request. In order to merge your pull request, you must:

* get the approval of an area lead (even though github will not enforce this)
* if you are the area lead get the approval of another area collaborator
* pass continuous integration tests (TravisCI)

The approver will leave a comment like "ready for merge" if they approve, but they will not approve until any other commented issues are resolved. They may approve and leave a "(nit)" comment in the case of extra spaces or misspellings; the writer is responsible for resolving these before merge but does not need new approval.

We use TravisCI to test that new high level code compiles. Eventually, it will run other unit tests too. Low level code is not checked by TravisCI. The approver is responsible for checking low level compilation, although the writer should check this too.

The goal of our branches/pull request policy is to keep ownership of the code with the writer while controlling the quality of what is in master. It is the writer's responsiblity to sheppard their code into master. This means they should not feel bad about bothering the necessary reviewers if they have not responded in more than a day.

## Area Leads

* High Level (`real_time/surface_src`): Trevor
* Low Level (`real_time/arduino_src`): Ian
* Tools (`offline`): Not as controlled. Anyone can check off.

## Notes

[1] Mergepocalypse resulted from running `git push -f` from a very old clone in May 2015. This dropped all changes from our master branch from Feb. to May. Because git is awesome, we could piece together most of the changes from other copies and branches, but master is still missing several months of history. We would like to not repeat the experience.
