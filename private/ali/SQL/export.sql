
SELECT person.id, person.name, person.birthday, motion.type, record.date_added, record.angles 

FROM record JOIN person ON (record.person=person.id) JOIN motion ON (record.type=motion.id)

ORDER BY person.name, person.birthday, motion.type, record.date_added;
